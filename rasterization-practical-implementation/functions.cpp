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
#include </usr/local/include/eigen/Eigen/Dense>


using Eigen::Matrix;

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
}

void assignPixelColour(
    const triangle &tri,
    const uint32_t &x, const uint32_t &y,
    setupParams &params,
    std::vector<float> &depthBuffer,
    std::vector<Vec3<unsigned char>> &frameBuffer
)
{
    Vec3f pixelSample(x + 0.5, y + 0.5, 0);
    float w0 = edgeFunction(tri.v1Raster, tri.v2Raster, pixelSample);
    float w1 = edgeFunction(tri.v2Raster, tri.v0Raster, pixelSample);
    float w2 = edgeFunction(tri.v0Raster, tri.v1Raster, pixelSample);
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