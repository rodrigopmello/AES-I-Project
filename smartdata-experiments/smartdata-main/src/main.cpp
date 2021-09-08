#include "main_traits.h"
#include <utility/observer.h>
#include <network/tstp/tstp.h>
#include <transducer.h>
#include <smartdata.h>
#include <unistd.h>
#include <string.h>
// #include <machine/udpnic.h>
// #include <network/tstp/tstp.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
  
#define PORT     5001
#define MAXLINE 1024


static const int ITERATIONS = 10;

#define Delay sleep

// This is not good, but it will work...
unsigned int globalCoord = 1;
const char* globalIPAddress = "127.0.0.1";

OStream cout;

// void controller(const Antigravity & in1, const Antigravity & in2, Antigravity & out) {}

void sink();
void Usage();
void node();
void server();

int main(int argc, char* argv[])
{
	cout << "SmartData Test" << endl;
	// NIC<NIC_Family>* nic = new UDPNIC();
	// Buffer * buffer = Network::alloc(sizeof(Response) + sizeof(Value));
	// nic->send()
	

	
	if (argc != 2)
	{
		Usage();
		return -1;
	}

	bool isSink;
	if (!strncmp(argv[1], "sink", 4))
		isSink = true;
	else if (!strncmp(argv[1], "node", 4))
		isSink = false;
	else
	{
		Usage();
		return -1;
	}
	
	TSTP::init();	
	cout << "Sizes:" << endl;	
	cout << "  SmartData::Unit:           " << sizeof(SmartData::Unit) << endl;
	cout << "  SmartData::Value<SI|I32>:  " << sizeof(SmartData::Value<SmartData::Unit::SI | SmartData::Unit::I32>) << endl;
	cout << "  SmartData::Value<SI|I64>:  " << sizeof(SmartData::Value<SmartData::Unit::SI | SmartData::Unit::I64>) << endl;
	cout << "  SmartData::Value<SI|F32>:  " << sizeof(SmartData::Value<SmartData::Unit::SI | SmartData::Unit::F32>) << endl;
	cout << "  SmartData::Value<SI|D64>:  " << sizeof(SmartData::Value<SmartData::Unit::SI | SmartData::Unit::D64>) << endl;
	cout << "  SmartData::Spacetime:      " << sizeof(SmartData::Spacetime) << endl;
	cout << "  SmartData::Space:          " << sizeof(SmartData::Space) << endl;
	cout << "  SmartData::Global_Space:   " << sizeof(SmartData::Global_Space) << endl;
	cout << "  SmartData::Time:           " << sizeof(SmartData::Time) << endl;
	cout << "  SmartData::Region:         " << sizeof(SmartData::Region) << endl;
	cout << "  SmartData::Header:         " << sizeof(SmartData::Header) << endl;
	cout << "  SmartData::Interest:       " << sizeof(SmartData::Interest) << endl;
	cout << "  SmartData::Response:       " << sizeof(SmartData::Response) << endl;
	cout << "  SmartData::Command:        " << sizeof(SmartData::Command) << endl;
	cout << "  SmartData::Control:        " << sizeof(SmartData::Control) << endl;	
	cout << "  TSTP::Header:              " << sizeof(TSTP::Header) << endl;
	cout << "  TSTP::Packet:              " << sizeof(TSTP::Packet) << endl;

	if (isSink)
		sink();
	else
		node();

	//    CAntigravity ctl(&controller, a, b, c);

	cout << "Bye!" << endl;

	return 0;
}

void Usage()
{
	cout << "Usage:" << endl;
	cout << "  smartdata <mode>" << endl;
	cout << "  mode: sink or node" << endl;
}


void sink()
{
	cout << "I'm the sink!" << endl;
	
	cout << "current time " << Antigravity::now() << endl;
	cout << "expiry time " << Antigravity::now() + (ITERATIONS + 5) * 1000 << endl;	
	Antigravity_Proxy a(Antigravity::Region(0, 0, 0, 100, Antigravity::now(), Antigravity::now() + (ITERATIONS + 500) * 100), 10000000);
	//    Smart_Key_Proxy d(Smart_Key::Region((0, 0, 0), 100, Smart_Key::now(), Smart_Key::now()+10000000), 10000000);
	
	cout << "My coordinates are " << a.here() << endl;
	cout << "The time now is " << a.now() << endl;

	cout << "I'm interested on " << SmartData::Unit(a.unit()) << endl;
	cout << "I'll wait for data of this kind for " << ITERATIONS << " seconds..." << endl;
	for (int i = 0; i < ITERATIONS + 5; i++) {
		cout << "a=" << a << endl;
		//server();
		Delay(10);
	}
	cout << "done!" << endl;

}


void node()
{
	cout << "I'm a node!" << endl;
	Delay(5);	
	Antigravity a(0, 1000000, SmartData::ADVERTISED);
	//    Antigravity b(0, 10000000, Antigravity::ADVERTISED);
	//    Antigravity c(0, 1000000, Antigravity::COMMANDED);

	//    Smart_Key d(0, 500000, Smart_Key::PRIVATE);

	cout << "My coordinates are " << a.here() << endl;
	cout << "The time now is " << a.now() << endl;

	//    a = 1;
	//    b = 1;
	//    c = 1;

	cout << "I have three sensors that measure " << SmartData::Unit(a.unit()) << endl;
	cout << "OMG that's ANTI-GRAVITY!!!" << endl;
	cout << "I'll update data of this kind for " << ITERATIONS << " seconds..." << endl;

	for (int i = 0; i < ITERATIONS; i++) {
		a = i;
		//        b = i * 2;
		//        c = i * 3;		
		// cout << "a=" << a << endl;
		//        cout << "b=" << b << endl;
		//        db<TSTP>(TRC) << "c=" << c << endl;
		cout << "waiting" << endl;
		Delay(10);
	}
	cout << "done!" << endl;
}
