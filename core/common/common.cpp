#include "common.hpp"

//const vector<Point2D> Landmarks::corners={{0,0},{290,0},{0,580},{880,580},{580,0}};
const vector<Point2D> Landmarks::corners={{0,0},{0,580},{880,580},{580,0},{1170,0},{1170,290}};
const vector<Point2D> Landmarks::edges={{290,290},{580,290},{880,290}};
constexpr u16 MazeDim::world_x_mm{1170};
constexpr u16 MazeDim::world_y_mm{580};
const GraphNodeList MazeGraph::nodes={
    {{0,0},{{0,1}}},
    {{0,1},{{0,0},{1,1}}},
    {{1,1},{{0,1},{2,1}}},
    {{2,1},{{1,1},{2,0}}},
    {{2,0},{{2,1},{3,0}}},
    {{3,0},{{2,0}}}
};
const GraphNode MazeGraph::goal={0,0};