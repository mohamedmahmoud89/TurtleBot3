#include "common.hpp"

const vector<Point2D> Landmarks::corners={{0,0},{290,0},{0,580},{880,580},{580,0},{880,0}};
const vector<Point2D> Landmarks::edges={{290,290},{580,290}};
const GraphNodeList MazeGraph::nodes={
    {{0,0},{{0,1}}},
    {{0,1},{{0,0},{1,1}}},
    {{1,1},{{0,1},{2,1}}},
    {{2,1},{{1,1},{2,0}}},
    {{2,0},{{2,1}}}
};
const GraphNode MazeGraph::goal={2,0};