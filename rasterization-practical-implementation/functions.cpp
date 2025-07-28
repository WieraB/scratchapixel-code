// Copyright (C) 2012  www.scratchapixel.com
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// clang++ -o raster3d.exe raster3d.cpp -O3

#define _USE_MATH_DEFINES 

#include "geometry.h"
#include <fstream>
#include <chrono>
#include <memory>
#include <ranges>
#include <algorithm>
#include <array>
#include <vector>
#include <nlohmann/json.hpp>

#include "cow.h"

static const float inchToMm = 25.4;
enum FitResolutionGate { kFill = 0, kOverscan };

void computeScreenCoordinates(
    setupParams &params,
    const FitResolutionGate &fitFilm
)
{
    float filmAspectRatio = params.filmApertureWidth / params.filmApertureHeight;
    float deviceAspectRatio = params.imageWidth / (float)params.imageHeight;

    params.top = ((params.filmApertureHeight * inchToMm / 2) / params.focalLength) * params.nearClippingPlane;
    params.right = ((params.filmApertureWidth * inchToMm / 2) / params.focalLength) * params.nearClippingPlane;

    // field of view (horizontal)
    float fov = 2 * 180 / M_PI * atan((params.filmApertureWidth * inchToMm / 2) / params.focalLength);
    std::cerr << "Field of view " << fov << std::endl;
    
    float xscale = 1;
    float yscale = 1;
    
    switch (fitFilm) {
        default:
        case kFill:
            if (filmAspectRatio > deviceAspectRatio) {
                xscale = deviceAspectRatio / filmAspectRatio;
            }
            else {
                yscale = filmAspectRatio / deviceAspectRatio;
            }
            break;
        case kOverscan:
            if (filmAspectRatio > deviceAspectRatio) {
                yscale = filmAspectRatio / deviceAspectRatio;
            }
            else {
                xscale = deviceAspectRatio / filmAspectRatio;
            }
            break;
    }

    params.right *= xscale;
    params.top *= yscale;

    params.bottom = -params.top;
    params.left = -params.right;
}

void convertPointToRaster(
    const Vec3f &vertexWorld,
    setupParams &params,
    Vec3f &vertexRaster
)
{
    Vec3f vertexCamera;

    params.worldToCamera.multVecMatrix(vertexWorld, vertexCamera);

    Vec2f vertexScreen;
    vertexScreen.x = params.nearClippingPlane * vertexCamera.x / -vertexCamera.z;
    vertexScreen.y = params.nearClippingPlane * vertexCamera.y / -vertexCamera.z;

    Vec2f vertexNDC;
    vertexNDC.x = 2 * vertexScreen.x / (params.right - params.left) - (params.right + params.left) / (params.right - params.left);
    vertexNDC.y = 2 * vertexScreen.y / (params.top - params.bottom) - (params.top + params.bottom) / (params.top - params.bottom);

    vertexRaster.x = (vertexNDC.x + 1) / 2 * params.imageWidth;
    vertexRaster.y = (1 - vertexNDC.y) / 2 * params.imageHeight;
    vertexRaster.z = -vertexCamera.z;
}

void convertTriangleToRaster(
    triangle &tri, setupParams &params
)
{
    convertPointToRaster(tri.v0, params, tri.v0Raster);
    convertPointToRaster(tri.v1, params, tri.v1Raster);
    convertPointToRaster(tri.v2, params, tri.v2Raster);
}


float min3(const float &a, const float &b, const float &c)
{ return std::ranges::min(std::array{a, b, c}); } 

float max3(const float &a, const float &b, const float &c)
{ return std::ranges::max(std::array{a, b, c}); }

float edgeFunction(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{ return (c[0] - a[0]) * (b[1] - a[1]) - (c[1] - a[1]) * (b[0] - a[0]); }

void projectTriangleToRaster(
    triangle &tri, setupParams &params
)
{
    convertTriangleToRaster(tri, params);

    tri.v0Raster.z = 1 / tri.v0Raster.z;
    tri.v1Raster.z = 1 / tri.v1Raster.z;
    tri.v2Raster.z = 1 / tri.v2Raster.z;

    tri.st0 *= tri.v0Raster.z;
    tri.st1 *= tri.v1Raster.z;
    tri.st2 *= tri.v2Raster.z;

    float xmin = min3(tri.v0Raster.x, tri.v1Raster.x, tri.v2Raster.x);
    float ymin = min3(tri.v0Raster.y, tri.v1Raster.y, tri.v2Raster.y);
    float xmax = max3(tri.v0Raster.x, tri.v1Raster.x, tri.v2Raster.x);
    float ymax = max3(tri.v0Raster.y, tri.v1Raster.y, tri.v2Raster.y);

    if (xmin > params.imageWidth - 1 || xmax < 0 || ymin > params.imageHeight - 1 || ymax < 0) {
        throw std::invalid_argument("Triangle is not visible (outside the domain).");
    }

    tri.x0 = std::max(int32_t(0), (int32_t)(std::floor(xmin)));
    tri.x1 = std::min(int32_t(params.imageWidth) - 1, (int32_t)(std::floor(xmax)));
    tri.y0 = std::max(int32_t(0), (int32_t)(std::floor(ymin)));
    tri.y1 = std::min(int32_t(params.imageHeight) - 1, (int32_t)(std::floor(ymax)));

    tri.area = edgeFunction(tri.v0Raster, tri.v1Raster, tri.v2Raster);
    tri.edge1.y = tri.v2Raster[1] - tri.v1Raster[1];
    tri.edge1.x = tri.v2Raster[0] - tri.v1Raster[0];
    tri.edge2.y = tri.v0Raster[1] - tri.v2Raster[1];
    tri.edge2.x = tri.v0Raster[0] - tri.v2Raster[0];
    tri.edge0.y = tri.v1Raster[1] - tri.v0Raster[1];
    tri.edge0.x = tri.v1Raster[0] - tri.v0Raster[0];
}

void assignPixelColour(
    const triangle &tri,
    const uint32_t &j, const uint32_t &i,
    const uint32_t &x, const uint32_t &y,
    std::vector<float> &w,
    setupParams &params,
    std::vector<float> &depthBuffer,
    std::vector<Vec3<unsigned char>> &frameBuffer
)
{
    float w0 = w[0];
    float w1 = w[1];
    float w2 = w[2];

    if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
        w0 /= tri.area;
        w1 /= tri.area;
        w2 /= tri.area;
        float oneOverZ = tri.v0Raster.z * w0 + tri.v1Raster.z * w1 + tri.v2Raster.z * w2;
        float z = 1 / oneOverZ;

        if (z < depthBuffer[y * params.imageWidth + x]) {
            depthBuffer[y * params.imageWidth + x] = z;

            Vec2f st = tri.st0 * w0 + tri.st1 * w1 + tri.st2 * w2;

            st *= z;
            
            Vec3f v0Cam, v1Cam, v2Cam;
            params.worldToCamera.multVecMatrix(tri.v0, v0Cam);
            params.worldToCamera.multVecMatrix(tri.v1, v1Cam);
            params.worldToCamera.multVecMatrix(tri.v2, v2Cam);

            float px = (v0Cam.x/-v0Cam.z) * w0 + (v1Cam.x/-v1Cam.z) * w1 + (v2Cam.x/-v2Cam.z) * w2;
            float py = (v0Cam.y/-v0Cam.z) * w0 + (v1Cam.y/-v1Cam.z) * w1 + (v2Cam.y/-v2Cam.z) * w2;
            
            Vec3f pt(px * z, py * z, -z); // pt is in camera space
            
            Vec3f n = (v1Cam - v0Cam).crossProduct(v2Cam - v0Cam);
            n.normalize();
            Vec3f viewDirection = -pt;
            viewDirection.normalize();
            float nDotView =  std::max(0.f, n.dotProduct(viewDirection));
            
            const int M = 10;
            float checker = (fmod(st.x * M, 1.0) > 0.5) ^ (fmod(st.y * M, 1.0) < 0.5);
            float c = 0.3 * (1 - checker) + 0.7 * checker;
            nDotView *= c;
            frameBuffer[y * params.imageWidth + x].x = nDotView * 255;
            frameBuffer[y * params.imageWidth + x].y = nDotView * 255;
            frameBuffer[y * params.imageWidth + x].z = nDotView * 255;

        }
    }

}

void assignPixelColoursToTile(
    const std::vector<triangle>& triangles, 
    setupParams &params,
    const std::vector<int> &tileBin,
    const uint32_t xStart, const uint32_t yStart,
    const uint32_t xEnd, const uint32_t yEnd,
    std::vector<float> &depthBuffer,
    std::vector<Vec3<unsigned char>> &frameBuffer
)
{
    for (int i : tileBin) {
        const triangle& tri = triangles[i];
        if (!tri.visibility) continue;

        uint32_t y0Pix = std::max(tri.y0, yStart);
        uint32_t y1Pix = std::min(tri.y1, yEnd - 1);
        uint32_t x0Pix = std::max(tri.x0, xStart);
        uint32_t x1Pix = std::min(tri.x1, xEnd - 1);

        Vec3f pixelSample0(x0Pix + 0.5, y0Pix + 0.5, 0);
        std::vector<float> wxCount(3);
        std::vector<float> wyCount(3);
        wyCount[0] = edgeFunction(tri.v1Raster, tri.v2Raster, pixelSample0) ;
        wyCount[1] = edgeFunction(tri.v2Raster, tri.v0Raster, pixelSample0);
        wyCount[2] = edgeFunction(tri.v0Raster, tri.v1Raster, pixelSample0);

        for (uint32_t i = 0; i <= y1Pix - y0Pix; ++i) {
            wxCount[0] = wyCount[0];
            wxCount[1] = wyCount[1];
            wxCount[2] = wyCount[2];
            for (uint32_t j = 0; j <= x1Pix - x0Pix; ++j) {
                uint32_t x = x0Pix + j;
                uint32_t y = y0Pix + i;
                assignPixelColour(tri, j, i, x, y, wxCount, params, depthBuffer, frameBuffer);
                wxCount[0] += tri.edge1.y;
                wxCount[1] += tri.edge2.y;
                wxCount[2] += tri.edge0.y;
            }
            wyCount[0] -= tri.edge1.x;
            wyCount[1] -= tri.edge2.x;
            wyCount[2] -= tri.edge0.x;
        }
    }
}

void projectTriangleToRasterRange(
    std::vector<triangle>& triangles, 
    setupParams &params, 
    const uint32_t start, const uint32_t end,
    std::vector<std::vector<std::vector<int>>> &tileBins,
    const int tilesX, const int tilesY, const int tileSize,
    const std::mutex *m
) 
{
    for (uint32_t i = start; i < end; ++i) {
        triangle tri;

        tri.v0 = vertices[nvertices[i * 3]];
        tri.v1 = vertices[nvertices[i * 3 + 1]];
        tri.v2 = vertices[nvertices[i * 3 + 2]];

        tri.st0 = st[stindices[i * 3]];
        tri.st1 = st[stindices[i * 3 + 1]];
        tri.st2 = st[stindices[i * 3 + 2]];
            
        try {
            tri.visibility = true;
            projectTriangleToRaster(tri, params);
        } catch (const std::invalid_argument &e) {
            tri.visibility = false;
            continue;
        }

        triangles[i] = tri;

        int x0 = tri.x0 / tileSize;
        int x1 = tri.x1 / tileSize;
        int y0 = tri.y0 / tileSize;
        int y1 = tri.y1 / tileSize;
        
        for (int ty = y0; ty <= y1; ++ty) { // Loop through tiles in Y direction
            for (int tx = x0; tx <= x1; ++tx) { // Loop through tiles in X direction
                tileBins[ty][tx].push_back(i);
            }
        }

    }
}

void assignPixelColoursToTileRange(
    const std::vector<triangle>& triangles, 
    setupParams &params, 
    const std::vector<std::vector<std::vector<int>>> &tileBins,
    const std::vector<std::pair<int, int>> &tileJobs,
    uint32_t start, uint32_t end,
    const int tileSize,
    std::vector<float> &depthBuffer,
    std::vector<Vec3<unsigned char>> &frameBuffer
) 
{
    for (uint32_t i = start; i < end; ++i) {
            int ty = tileJobs[i].first;
            int tx = tileJobs[i].second;

            uint32_t xStart = tx * tileSize;
            uint32_t yStart = ty * tileSize;
            uint32_t xEnd = std::min(xStart + tileSize, params.imageWidth);
            uint32_t yEnd = std::min(yStart + tileSize, params.imageHeight);

            assignPixelColoursToTile(triangles, params, tileBins[ty][tx], xStart, yStart, xEnd, yEnd, depthBuffer, frameBuffer);
    }
}

void saveOutput(
    const std::string& filename,
    const setupParams &params, 
    std::vector<Vec3<unsigned char>> &frameBuffer
) 
{
    std::ofstream ofs(filename, std::ios::binary);
    ofs << "P6\n" << params.imageWidth << " " << params.imageHeight << "\n255\n";
    ofs.write(reinterpret_cast<char*>(frameBuffer.data()), params.imageArea * 3);
    ofs.close();
}

using json = nlohmann::json;
void readInput(
    const std::string& filename,
    setupParams &params
) 
{
    std::ifstream in(filename);
    json j;
    in >> j;

    params.imageWidth = j["imageWidth"];
    params.imageHeight = j["imageHeight"];
    params.nearClippingPlane = j["nearClippingPlane"];
    params.farClippingPLane = j["farClippingPlane"];
    params.focalLength = j["focalLength"];
    params.filmApertureWidth = j["filmApertureWidth"];
    params.filmApertureHeight = j["filmApertureHeight"];
    std::array<float, 16> flat = j["worldToCamera"];
    params.worldToCamera = Matrix44f(
        flat[0], flat[1], flat[2], flat[3],
        flat[4], flat[5], flat[6], flat[7],
        flat[8], flat[9], flat[10], flat[11],
        flat[12], flat[13], flat[14], flat[15]
    );
    params.numThreadsTriangles = j["numThreadsTriangles"];
    params.numThreadsTiles = j["numThreadsTiles"];
    params.tileSize = j["tileSize"];
}


std::streampos skipPPMHeader(std::ifstream& file) {
    std::string line;
    int headerLines = 0;
    while (headerLines < 3 && std::getline(file, line)) {
        if (!line.empty() && line[0] != '#') {
            ++headerLines;
        }
    }
    return file.tellg(); // return position after header
}


bool comparePPMFiles(const std::string& file1, const std::string& file2) {

    std::ifstream f1(file1, std::ios::binary);
    std::ifstream f2(file2, std::ios::binary);

    if (!f1 || !f2) {
        std::cerr << "Failed to open one or both files.\n";
        return false;
    }

    std::streampos dataStart1 = skipPPMHeader(f1);
    std::streampos dataStart2 = skipPPMHeader(f2);

    f1.seekg(0, std::ios::end);
    f2.seekg(0, std::ios::end);
    std::streampos size1 = f1.tellg() - dataStart1;
    std::streampos size2 = f2.tellg() - dataStart2;

    if (size1 != size2) {
        std::cerr << "Different image sizes or headers.\n";
    }

    f1.seekg(dataStart1);
    f2.seekg(dataStart2);

    char byte1, byte2;
    size_t mismatchCount = 0;
    size_t totalBytes = size1;

    for (size_t i = 0; i < totalBytes; ++i) {
        f1.read(&byte1, 1);
        f2.read(&byte2, 1);
        if (byte1 != byte2) {
            ++mismatchCount;
        }
    }

    if (mismatchCount == 0) {
        std::cout << "PPM files match.\n";
        return true;
    } else {
        std::cout << "PPM files differ in " << mismatchCount << " bytes.\n";
        return false;
    }
}
