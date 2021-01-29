#ifndef LFX_ITF_HPP
#define LFX_ITF_HPP

#include "funcItf.hpp"
#include "common.hpp"

using namespace std;

class LfxInputData : public FuncItf{
public:
    LfxInputData():pos(0,0,M_PI_2),scan(360,0){}
    RobotPos pos;
    vector<u16> scan;
};

class LfxOutputData : public FuncItf{
public:
    LfxFeats feats;
};

#endif