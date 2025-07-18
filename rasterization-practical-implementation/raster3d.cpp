// Copyright (C) 2012  www.scratchapixel.com
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// clang++ -o raster3d.exe raster3d.cpp -O3

#define _USE_MATH_DEFINES 

#include <fstream>
#include <chrono>
#include <memory>
#include <ranges>
#include <algorithm>
#include <array>
#include <vector>
#include "functions.cpp"

#include "cow.h"

const uint32_t ntris = 3156;

int main(int argc, char **argv)
{
    setupParams params;
    
    params.imageWidth = 640;
    params.imageHeight = 480;
    params.imageArea = params.imageWidth * params.imageHeight;

    params.worldToCamera = {0.707107, -0.331295, 0.624695, 0, 0, 0.883452, 0.468521, 0, -0.707107, -0.331295, 0.624695, 0, -1.63871, -5.747777, -40.400412, 1};
    params.cameraToWorld = params.worldToCamera.inverse();

    params.nearClippingPlane = 1;
    params.farClippingPLane = 1000;
    params.focalLength = 20; // in mm

    params.filmApertureWidth = 0.980;
    params.filmApertureHeight = 0.735;

    computeScreenCoordinates(params, kOverscan);

    std::vector<Vec3<unsigned char>> frameBuffer(params.imageArea, Vec3<unsigned char>(255));
    std::vector<float> depthBuffer(params.imageArea, params.farClippingPLane);
    std::vector<float> areas(ntris, 0.0);

    auto t_start = std::chrono::high_resolution_clock::now();
    
    for (uint32_t i = 0; i < ntris; ++i) {
        triangle tri;
        tri.v0 = vertices[nvertices[i * 3]];
        tri.v1 = vertices[nvertices[i * 3 + 1]];
        tri.v2 = vertices[nvertices[i * 3 + 2]];

        tri.st0 = st[stindices[i * 3]];
        tri.st1 = st[stindices[i * 3 + 1]];
        tri.st2 = st[stindices[i * 3 + 2]];
        
        try {
            projectTriangleToRaster(tri, params);
        } catch (const std::invalid_argument &e) {
            continue;
        }

        for (uint32_t y = tri.y0; y <= tri.y1; ++y) {
            for (uint32_t x = tri.x0; x <= tri.x1; ++x) {
                assignPixelColour(tri, x, y, params, depthBuffer, frameBuffer);
            }
        }
    }
    
	auto t_end = std::chrono::high_resolution_clock::now();
	auto passedTime = std::chrono::duration<double, std::milli>(t_end - t_start).count();
	std::cerr << "Wall passed time: " << passedTime << "ms" << std::endl;
    
	std::ofstream ofs;
	ofs.open("./output.ppm", std::ios::binary);
	ofs << "P6\n" << params.imageWidth << " " << params.imageHeight << "\n255\n";
    ofs.write(reinterpret_cast<char*>(frameBuffer.data()), params.imageArea * 3);
	ofs.close();
    
    return 0;
}