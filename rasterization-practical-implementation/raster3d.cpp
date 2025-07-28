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
#include <thread>
#include <execution>
#include "functions.cpp"

const uint32_t ntris = 3156;

int main(int argc, char **argv)
{
    // Define parameters and vectors

    setupParams params;
    readInput("./input.json", params);
    params.imageArea = params.imageWidth * params.imageHeight;
    params.cameraToWorld = params.worldToCamera.inverse();

    computeScreenCoordinates(params, kOverscan);

    std::vector<Vec3<unsigned char>> frameBuffer(params.imageArea, Vec3<unsigned char>(255));
    std::vector<float> depthBuffer(params.imageArea, params.farClippingPLane);
    std::vector<triangle> triangles(ntris);

    int tilesX = (params.imageWidth + params.tileSize - 1) / params.tileSize; // Calculate number of tiles in X direction
    int tilesY = (params.imageHeight + params.tileSize - 1) / params.tileSize; // Calculate number of tiles in Y direction
    std::vector<std::vector<std::vector<int>>> tileBins(tilesY, std::vector<std::vector<int>>(tilesX));

    std::vector<std::pair<int, int>> tileJobs;
    for (int ty = 0; ty < tilesY; ++ty) {
        for (int tx = 0; tx < tilesX; ++tx) {
            tileJobs.emplace_back(ty, tx);
        }
    }
    uint32_t totalTiles = tileJobs.size();

    std::vector<std::thread> threadsTriangles;
    uint32_t sectionSizeTriangles = ntris / params.numThreadsTriangles;
    std::vector<std::thread> threadsTiles;
    uint32_t sectionSizeTiles = (totalTiles + params.numThreadsTiles - 1) / params.numThreadsTiles;
    std::mutex m;

    std::vector<std::vector<std::vector<std::vector<int>>>> threadLocalBins(
    params.numThreadsTriangles,
    std::vector<std::vector<std::vector<int>>>(
        tilesY, std::vector<std::vector<int>>(tilesX)
    )
    );

    auto t_start = std::chrono::high_resolution_clock::now();

    // Project triangles to raster space and assign projected triangles to tiles (tile-based rasterisation)
    // One thread per section of triangles is used

    for (unsigned int t = 0; t < params.numThreadsTriangles; ++t) {
        uint32_t start = t * sectionSizeTriangles;
        uint32_t end = (t == params.numThreadsTriangles - 1) ? ntris : start + sectionSizeTriangles;
        threadsTriangles.emplace_back(projectTriangleToRasterRange, 
            std::ref(triangles), std::ref(params), 
            start, end, 
            std::ref(threadLocalBins[t]), tilesX, tilesY, params.tileSize, 
            &m);
    }

    for (auto& t : threadsTriangles) {
        t.join();
    }

    // Merge thread-local bins into global tile bins
    for (int t = 0; t < params.numThreadsTriangles; ++t) {
        for (int ty = 0; ty < tilesY; ++ty) {
            for (int tx = 0; tx < tilesX; ++tx) {
                auto& local = threadLocalBins[t][ty][tx];
                auto& global = tileBins[ty][tx];
                global.insert(global.end(), local.begin(), local.end());
            }
        }
    }

    // Assign pixel colours to tiles, one thread per section of tiles is used

    for (unsigned int t = 0; t < params.numThreadsTiles; ++t) {
        uint32_t start = t * sectionSizeTiles;
        uint32_t end = std::min(start + sectionSizeTiles, totalTiles);

        threadsTiles.emplace_back(assignPixelColoursToTileRange, 
            std::ref(triangles), std::ref(params), 
            std::ref(tileBins), std::ref(tileJobs), 
            start, end, 
            params.tileSize,
            std::ref(depthBuffer), std::ref(frameBuffer)
        );
    }

    for (auto& t : threadsTiles) {
        t.join();
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    auto passedTime = std::chrono::duration<double, std::milli>(t_end - t_start).count();
	std::cerr << "Wall passed time: " << passedTime << "ms" << std::endl;
    
    // Save the output to a file
    std::string filename = "./output.ppm";
    saveOutput(filename, params, frameBuffer);


    comparePPMFiles("./output.ppm", "./output_reference.ppm");
    
    return 0;
}