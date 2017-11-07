#include "Definition.h"
#include "electronic_torus.h"
#include "global_data.h"
#include "time.h"
#include "stdio.h"
#include "math.h"

#include <iostream>

int sc_main(int argc, char* argv[]) {
	if (argc == 6){
		injection_rate = atof(argv[1]);
		NOC_WIDTH=atoi(argv[2]);
		NOC_HEIGHT=atoi(argv[3]);
		NOC_DEPTH=atoi(argv[4]);
		simulation_time=atoi(argv[5]);
		processor_no=NOC_WIDTH*NOC_HEIGHT*NOC_DEPTH;
		core_no=processor_no;
	}
	else {
		injection_rate = 0.1;
		NOC_WIDTH=8;
		NOC_HEIGHT=4;
		NOC_DEPTH=2;
		simulation_time=100000;
		processor_no=NOC_WIDTH*NOC_HEIGHT*NOC_DEPTH;
		core_no=processor_no;
	}	

	sc_clock clk("clk", 1, SC_NS, 0.5);
	electronic_chip *noc;
	noc = new electronic_chip("noc");
	noc->clk(clk);
	noc->init(clk);
	//cout<<"\nStarting Simulation\n\n";
	cout<<"\nStarting Simulation\n\n";
	sc_start(simulation_time, SC_NS);

//	sc_close_vcd_trace_file(tf);
	cout<<"\nTerminating Simulation\n\n";

	noc->check_finish();
	ofstream formatted_throughput_delay_log;
	ofstream throughput_delay_log;

	formatted_throughput_delay_log.open("uniform_formatted_3D_OMesh_throughput_delay.txt",ios::out|ios::app);

	formatted_throughput_delay_log<<injection_rate<<"	"<<noc->total_throughput<<"	"<<noc->average_pkt_delay<<"	";
	
	formatted_throughput_delay_log<<endl;
								

	formatted_throughput_delay_log.close();

	return 0;

}