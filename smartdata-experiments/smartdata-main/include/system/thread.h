#pragma once
#include <system/types.h>
#include <unistd.h>
#include <pthread.h>

class Thread
{
public:
	Thread(int (*)())
	{
		// TCB - to be implemented.
		db<Thread>(TRC) << "Thread::Thread()" << endl;
	}
	static void yield()
	{
		db<Thread>(TRC) << "Thread::yield()" << endl;
		usleep(100 * 1000); // TCB - avoid using excessive CPU. Maybe it should be removed at the end of implementation.
		pthread_yield();
	}
};

class Periodic_Thread : public Thread
{
private:
	Microsecond _period;

public:
	Periodic_Thread(const Microsecond&, int(*callback)(unsigned int, SmartData::Time, Responsive_SmartData<Dummy_Transducer>*),
		unsigned int&, const SmartData::Time&, Responsive_SmartData<Dummy_Transducer>*) : Thread::Thread(0)
	{
		// TCB - to be implemented.
		db<Periodic_Thread>(TRC) << "Periodic_Thread::Periodic_Thread()" << endl;
	}

	const Microsecond& period() { return _period; }
	void period(const Microsecond& p) { _period = p; }
	static volatile bool wait_next() {

		// TCB - to be implemented.
		db<Periodic_Thread>(TRC) << "Periodic_Thread::wait_next()" << endl;
		return true;
	}
};

class Alarm
{
public:
	Alarm(const Microsecond & time, Handler * handler, unsigned int times = 1)
	{
		// TCB - to be implemented.
		db<Alarm>(TRC) << "Alarm::Alarm()" << endl;
	}
	static void delay(const Microsecond & time)
	{
		// TCB - to be implemented.
		db<Alarm>(TRC) << "Alarm::delay()" << endl;
	}
	void reset()
	{
		// TCB - to be implemented.
		db<Alarm>(TRC) << "Alarm::reset()" << endl;
	}
};