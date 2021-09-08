#pragma once

// EPOS IA32 Time-Stamp Counter Mediator Declarations
#include <chrono>
#include <architecture/cpu.h>
#include <architecture/tsc.h>
#include <system/types.h>



class TSC: private TSC_Common
{
public:
    using TSC_Common::Time_Stamp;

public:
    TSC() {}

    static Hertz frequency() { return CPU::clock(); }
    static PPB accuracy() { return 50; }

    static Time_Stamp time_stamp() {
        // Time_Stamp ts;
        // asm("rdtsc" : "=A" (ts) : ); // must be volatile!
        // return ts;
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
         
    }

    static void time_stamp(const Time_Stamp & ts) {
        CPU::wrmsr(CPU::MSR_TSC, ts);
    }
};
