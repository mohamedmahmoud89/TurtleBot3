#ifndef FUNC_BASE_HPP
#define FUNC_BASE_HPP

#include "funcItf.hpp"

class FuncBase{
protected:
    const FuncItf& inputs;
    FuncItf& outputs;
public:
    FuncBase(const FuncItf& in_ref,FuncItf& out_ref):
        inputs(in_ref),outputs(out_ref){}
    
    virtual ~FuncBase(){}
    virtual void exec() = 0;
};

#endif