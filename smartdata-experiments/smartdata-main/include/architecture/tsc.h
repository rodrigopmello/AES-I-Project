#pragma once

// EPOS TSC Mediator Common Package


class TSC_Common
{
protected:
    TSC_Common() {}

public:
    typedef unsigned long long Time_Stamp;
};

// __END_SYS

#if defined(__TSC_H) && !defined(__tsc_common_only__)
#include __TSC_H
#endif
