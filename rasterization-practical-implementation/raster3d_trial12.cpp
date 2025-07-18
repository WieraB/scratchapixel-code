// Copyright (C) 2012  www.scratchapixel.com
// Distributed under the terms of the CC BY-NC-ND 4.0 License.
// https://creativecommons.org/licenses/by-nc-nd/4.0/
// clang++ -o raster3d.exe raster3d.cpp -O3

#define _USE_MATH_DEFINES 

// #include "geometry.h"
#include <fstream>
#include <chrono>
#include <memory>
#include <ranges>
#include <algorithm>
#include <array>
#include <vector>
#include <thread>
#include <execution>
#include "functions_new.cpp"

// #include "cow.h"

const uint32_t ntris = 3156;

int main(int argc, char **argv)
{
    // Define parameters and vectors

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
    std::vector<triangle> triangles(ntris);

    const int tileSize = 32;
    int tilesX = (params.imageWidth + tileSize - 1) / tileSize; 
    int tilesY = (params.imageHeight + tileSize - 1) / tileSize;
    std::vector<std::vector<std::vector<int>>> tileBins(tilesY, std::vector<std::vector<int>>(tilesX));

    std::vector<std::pair<int, int>> tileJobs;
    for (int ty = 0; ty < tilesY; ++ty) {
        for (int tx = 0; tx < tilesX; ++tx) {
            tileJobs.emplace_back(ty, tx);
        }
    }
    uint32_t totalTiles = tileJobs.size();

    const unsigned int numThreadsTriangles = 16;
    std::vector<std::thread> threadsTriangles;
    uint32_t sectionSizeTriangles = ntris / numThreadsTriangles;
    const unsigned int numThreadsTiles = 16;
    std::vector<std::thread> threadsTiles;
    uint32_t sectionSizeTiles = (totalTiles + numThreadsTiles - 1) / numThreadsTiles;
    std::mutex m;

    std::vector<std::vector<std::vector<std::vector<int>>>> threadLocalBins(
    numThreadsTriangles,
    std::vector<std::vector<std::vector<int>>>(
        tilesY, std::vector<std::vector<int>>(tilesX)
    )
    );

    auto t_start = std::chrono::high_resolution_clock::now();

    // Project triangles to raster space and assign projected triangles to tiles (tile-based rasterisation)
    // One thread per section of triangles is used

    for (unsigned int t = 0; t < numThreadsTriangles; ++t) {
        uint32_t start = t * sectionSizeTriangles;
        uint32_t end = (t == numThreadsTriangles - 1) ? ntris : start + sectionSizeTriangles;
        threadsTriangles.emplace_back(projectTriangleToRasterRange, 
            std::ref(triangles), std::ref(params), 
            start, end, 
            std::ref(threadLocalBins[t]), tilesX, tilesY, tileSize, 
            &m);
    }

    for (auto& t : threadsTriangles) {
        t.join();
    }

    // Merge thread-local bins into global tile bins
    for (int t = 0; t < numThreadsTriangles; ++t) {
        for (int ty = 0; ty < tilesY; ++ty) {
            for (int tx = 0; tx < tilesX; ++tx) {
                auto& local = threadLocalBins[t][ty][tx];
                auto& global = tileBins[ty][tx];
                global.insert(global.end(), local.begin(), local.end());
            }
        }
    }

    // Assign pixel colours to tiles, one thread per section of tiles is used

    for (unsigned int t = 0; t < numThreadsTiles; ++t) {
        uint32_t start = t * sectionSizeTiles;
        uint32_t end = std::min(start + sectionSizeTiles, totalTiles);

        threadsTiles.emplace_back(assignPixelColoursToTileRange, 
            std::ref(triangles), std::ref(params), 
            std::ref(tileBins), std::ref(tileJobs), 
            start, end, 
            tileSize,
            std::ref(depthBuffer), std::ref(frameBuffer)
        );
    }

    for (auto& t : threadsTiles) {
        t.join();
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