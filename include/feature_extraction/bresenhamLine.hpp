/*******************************************************************************
 * File:        bresenhamLine.hpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     09/25/2022
 *
 * Description: Execute Bresenham's line algorithm
 *              Compute Grid Indexes that approximately lie on the line.
*******************************************************************************/
#ifndef BRESENHAM_LINE_HPP
#define BRESENHAM_LINE_HPP

// STD Library
#include <vector>
#include <utility>
#include <cmath>

inline void bresenhamLine(double start_x, double start_y,
                          double end_x, double end_y,
                          double grid_length, 
                          std::vector<std::pair<int, int>>& out_grid_idxs)
{
    out_grid_idxs.clear();
    
    int x0 = std::floor((start_x + grid_length / 2) / grid_length);
    int y0 = std::floor((start_y + grid_length / 2) / grid_length);
    int x1 = std::floor((end_x + grid_length / 2) / grid_length);
    int y1 = std::floor((end_y + grid_length / 2) / grid_length);
    
    double dx = std::abs(x1 - x0);
    double sx = (x0 < x1) ? 1 : -1;
    double dy = -std::abs(y1 - y0);
    double sy = (y0 < y1) ? 1 : -1;
    double error = dx + dy;

    while (true)
    {
        out_grid_idxs.push_back({x0, y0});
        if (x0 == x1 && y0 == y1)
        {
            break;
        }

        double e2 = 2 * error;
        if (e2 >= dy)
        {
            if (x0 == x1)
            {
                break;
            }
            error += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            if (y0 == y1)
            {
                break;
            }
            error += dx;
            y0 += sy;
        }
    }
}
#endif /* BRESENHAM_LINE_HPP */
