#pragma once
#include <system/traits.h>

// Build
template<> struct Traits<Build> : public Traits_Tokens
{
	// Basic configuration
	static const unsigned int MODE = LIBRARY;
	static const unsigned int ARCHITECTURE = RV32;
	static const unsigned int MACHINE = RISCV;
	static const unsigned int MODEL = SiFive_E;
	static const unsigned int CPUS = 1;
	static const unsigned int NODES = 1; // (> 1 => NETWORKING)
	static const unsigned int EXPECTED_SIMULATION_TIME = 60; // s (0 => not simulated)

	// Default flags
	static const bool enabled = true;
	static const bool monitored = false;
	static const bool debugged = true;
	static const bool hysterically_debugged = false;

	// Default aspects
	typedef ALIST<> ASPECTS;
};

#include <architecture/ia32/ia32_traits.h>

class Machine_Common;
template<> struct Traits<Machine_Common> : public Traits<Build> {};

template<> struct Traits<Machine> : public Traits<Machine_Common>
{
	static const bool cpus_use_local_timer = false;

	static const unsigned int NOT_USED = 0xffffffff;
	static const unsigned int CPUS = Traits<Build>::CPUS;

	// Boot Image
	static const unsigned int BOOT_LENGTH_MIN = 512;
	static const unsigned int BOOT_LENGTH_MAX = 512;
	static const unsigned int BOOT_IMAGE_ADDR = 0x00008000;
	static const unsigned int RAMDISK = 0x0fa28000; // MEMDISK-dependent
	static const unsigned int RAMDISK_SIZE = 0x003c0000;

	// Physical Memory
	static const unsigned int MEM_BASE = 0x00000000;
	static const unsigned int MEM_TOP = 0x10000000; // 256 MB (MAX for 32-bit is 0x70000000 / 1792 MB)
	static const unsigned int BOOT_STACK = NOT_USED;   // not used (defined by BOOT and by SETUP)

	// Logical Memory Map
	static const unsigned int BOOT = 0x00007c00;
	static const unsigned int SETUP = 0x00100000; // 1 MB
	static const unsigned int INIT = 0x00200000; // 2 MB

	static const unsigned int APP_LOW = 0x00000000;
	static const unsigned int APP_CODE = 0x00000000;
	static const unsigned int APP_DATA = 0x00400000; // 4 MB
	static const unsigned int APP_HIGH = 0x0fffffff; // 256 MB

	static const unsigned int PHY_MEM = 0x80000000; // 2 GB
	static const unsigned int IO_BASE = 0xf0000000; // 4 GB - 256 MB
	static const unsigned int IO_TOP = 0xff400000; // 4 GB - 12 MB

	static const unsigned int SYS = IO_TOP;     // 4 GB - 12 MB
	static const unsigned int SYS_CODE = 0xff700000;
	static const unsigned int SYS_DATA = 0xff740000;

	// Default Sizes and Quantities
	static const unsigned int STACK_SIZE = 16 * 1024;
	static const unsigned int HEAP_SIZE = 16 * 1024 * 1024;
	static const unsigned int MAX_THREADS = 16;
};

// API Components
template<> struct Traits<Application> : public Traits<Build>
{
	static const unsigned int STACK_SIZE = Traits<Machine>::STACK_SIZE;
	static const unsigned int HEAP_SIZE = Traits<Machine>::HEAP_SIZE;
	static const unsigned int MAX_THREADS = Traits<Machine>::MAX_THREADS;
};

template<> struct Traits<System> : public Traits<Build>
{
	static const unsigned int mode = Traits<Build>::MODE;
	static const bool multithread = (Traits<Build>::CPUS > 1) || (Traits<Application>::MAX_THREADS > 1);
	static const bool multitask = (mode != Traits<Build>::LIBRARY);
	static const bool multicore = (Traits<Build>::CPUS > 1) && multithread;
	static const bool multiheap = multitask || Traits<Scratchpad>::enabled;

	static const unsigned long LIFE_SPAN = 1 * YEAR; // s
	static const unsigned int DUTY_CYCLE = 1000000; // ppm

	static const bool reboot = false;

	static const unsigned int STACK_SIZE = Traits<Machine>::STACK_SIZE;
	static const unsigned int HEAP_SIZE = (Traits<Application>::MAX_THREADS + 1) * Traits<Application>::STACK_SIZE;
};

template<> struct Traits<Thread> : public Traits<Build>
{
	static const bool enabled = Traits<System>::multithread;
	static const bool smp = Traits<System>::multicore;
	static const bool simulate_capacity = false;
	static const bool trace_idle = hysterically_debugged;

	typedef Priority Criterion;
	static const unsigned int QUANTUM = 10000; // us
};

template<> struct Traits<SmartData> : public Traits<Build>
{
	static const unsigned char PREDICTOR = NONE;
};

// Utilities
template<> struct Traits<Debug> : public Traits<Build>
{
	static const bool error = true;
	static const bool warning = true;
	static const bool info = true;
	static const bool trace = true;
};

template<> struct Traits<Network> : public Traits<Build>
{
	typedef LIST<TSTP> NETWORKS;

	static const unsigned int RETRIES = 3;
	static const unsigned int TIMEOUT = 10; // s

	static const bool enabled = (Traits<Build>::NODES > 1) && (NETWORKS::Length > 0);
};

template<> struct Traits<TSTP> : public Traits<Network>
{
	typedef Ethernet NIC_Family;
	static constexpr unsigned int NICS[] = { 0 }; // relative to NIC_Family (i.e. Traits<Ethernet>::DEVICES[NICS[i]]
	static const unsigned int UNITS = COUNTOF(NICS);

	static const unsigned int KEY_SIZE = 16;
	static const unsigned int RADIO_RANGE = 8000; // approximated radio range in centimeters

	static const bool enabled = Traits<Network>::enabled && (NETWORKS::Count<TSTP>::Result > 0);
};
