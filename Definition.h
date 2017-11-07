#ifndef electronic_torus_definition
#define electronic_torus_definition

#include "systemc.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include "time.h"
#include "stdlib.h"

const bool MY_PRINT=false;
const bool MY_DEBUG=false;

using namespace std;
const int MAX_LOOP_NUMBER_ALLOWED=100;
const double PI=std::atan(1.0)*4;
#define PRINT 0
// mesh-based topology
int NOC_WIDTH = 4;
int NOC_HEIGHT = 4;
int NOC_DEPTH = 2;
int processor_no=NOC_WIDTH*NOC_HEIGHT*NOC_DEPTH;
int core_no=processor_no;

#define PROCESSOR_NUM       1
#define CLUSTER_PORTS       7
#define inputbuffersize     3
#define ROUTING_ALG			XYZ_ROUTING

#define unfolded_long_link_delay     1 //here we assume mesh
#define unfolded_normal_link_delay   1
#define folded_normal_link_delay     2
#define folded_short_link_delay      1


#define X_change_road_threshold NOC_WIDTH/2+1
#define Y_change_road_threshold NOC_HEIGHT/2+1
#define Z_change_road_threshold NOC_DEPTH/2+1
const int port_no=7;//torus or mesh
// definition of ports in a router
#define LEFT				1
#define RIGHT				2
#define UP					3
#define DOWN				4
#define TSV_UP				5
#define TSV_DOWN			6
//add for delay module
#define left_right			1
#define right_left			2
#define down_up				3
#define up_down				4
#define TSV_down_up			5
#define TSV_up_down			6
//if whether_unfolded == true or false
#define whether_unfolded	1

#define TO_PROCESSOR		0
#define CLOCK_CYCLE			1
const int flit_size=32;
const int max_link_delay=100;
const int infinite=3000;//should not be too large!!!!, else exception will be caught

const int vc_number=1;
const int vc_depth=16; //for 16x16 unfolded torus, the long link delay is 15 cycles, this need to send buffer full when left space <=29

const int routing_algorithm_delay=1;

int traffic_model=0;
int execution_iterations=10;
const double XbarPower=0.07;
const double BufferPower=0.0015;
const double Dynamic_power=0.0015;
const double RouterControlPower=1.8;
const double LocalConnectPower=0.04;
const double GlobalConnectPower=0.62;
const double BusPower_unit=3.6/2;//10
int		scaling_factor;		// scale task execution time by multiplying by the factor
int start_record_iteration;
bool START_RECORD=false;
bool END_RECORD=false;
int end_record_iteration;
char *model_file_name;

int const_pkt_length=128;
//enum processor_core_type{core_idle,core_waiting_ };



struct flit_type
{
       int src; //the src and dest of cores id of each packet: information for routing
       int dest;

	   int src_task; //the src and dest task id of each packet : information for application flow
	   int dest_task;

	   bool control_type;// control pkt type notice
	   int consumed_token;//feedback information, telling the sender that one token is consumed 
	   int available_buffer_size;//feedback information, telling the sender how many buffers are aviable now

	   int edge_id;//the edge in the appilcation 

	   int src_chip;
	   int dest_chip;
	   int token_id; // this id is for token on a specific edge
	   int packet_length; //in flits
	   int token_length_in_pkt;// one token may be packeted in many packets
	   int token_length_in_flit;
	   int rank_in_token;//the offset of packets in the same token
	   int packet_sequence;
	   int generate_time;
	   int direction;
	   double power;

	   double switching_capacity_consumed;
	   int encountered_router_number;

	   int vc_selection;//add for torus deadlock break

	   int waiting_time;
	   int from_node_level;
	   int from_node_rank;

	   bool head;
	   bool tail;
	   int routing_delay;

       flit_type& operator=(const flit_type&);
       bool operator==(const flit_type&) const;
};




inline flit_type& flit_type::operator =(const flit_type &arg)
{
       src=arg.src;
       dest=arg.dest;

	   src_task=arg.src_task;
	   dest_task=arg.dest_task;

	   control_type=arg.control_type;
	   consumed_token=arg.consumed_token;
	   available_buffer_size=arg.available_buffer_size;

	   edge_id=arg.edge_id;
	   src_chip=arg.src_chip;
	   dest_chip=arg.dest_chip;
	   token_id=arg.token_id;
	   token_length_in_pkt=arg.token_length_in_pkt;
	   token_length_in_flit=arg.token_length_in_flit; 
	   rank_in_token=arg.rank_in_token;  
	 
	   packet_length=arg.packet_length;
	   packet_sequence=arg.packet_sequence;
	   generate_time=arg.generate_time;
	   head=arg.head;
	   tail=arg.tail;
	   direction=arg.direction;
	   power=arg.power;
	   routing_delay=arg.routing_delay;
	   switching_capacity_consumed=arg.switching_capacity_consumed;
	   encountered_router_number=arg.encountered_router_number;

	   vc_selection=arg.vc_selection;//add for torus vc selection

	   waiting_time=arg.waiting_time;
	   from_node_level=arg.from_node_level;
	   from_node_rank=arg.from_node_rank;

       return (*this);
}



inline bool flit_type::operator ==(const flit_type &arg) const
{
	return (src==arg.src)&&(dest==arg.dest)&&(src_task==arg.src_task) && (dest_task==arg.dest_task) 
		&&(src_chip==arg.src_chip)&&(dest_chip==arg.dest_chip)&&
		(packet_length==arg.packet_length)&&(edge_id == arg.edge_id) &&
		(packet_sequence==arg.packet_sequence)&&(generate_time==arg.generate_time)&&
		(head==arg.head)&&(tail==arg.tail)&&(direction==arg.direction)&&(power==arg.power)&&
		(switching_capacity_consumed==arg.switching_capacity_consumed)&&
		(encountered_router_number==arg.encountered_router_number)&&(vc_selection==arg.vc_selection)&& (token_length_in_pkt==arg.token_length_in_pkt)&&
		(token_length_in_flit==arg.token_length_in_flit)&& 
	   (rank_in_token==arg.rank_in_token)&&(token_id==arg.token_id)&&(control_type==arg.control_type) &&
	   (consumed_token==arg.consumed_token)&&
	   (available_buffer_size==arg.available_buffer_size);  
}




inline ostream&
operator<<(ostream& os, const flit_type& arg)
{
	os<<"src="<<arg.src<<" dest="<<arg.dest<<endl;
       return os;
}

extern  void sc_trace(sc_trace_file *tf, const flit_type& arg, const  std::string& name) 
{
       sc_trace(tf,arg.src,"src");//,name ".src");
       sc_trace(tf,arg.dest,"dest");//,name + ".dest");
	   sc_trace(tf,arg.packet_sequence,"seq");
}
struct generator_buffer_type{
	int						time_to_generate_pkt;
	flit_type				pkt_to_generate;						
};

struct output_buffer_type{
	flit_type				pkt_to_send;
};


struct Request_buffer
	{
		int src;
		int dest;
		bool wait_req_flag;
		bool wait_tear_flag;
	};

struct router_forward_interface
{
	bool ready;
	int vc_id;
	flit_type data_flit;
	router_forward_interface& operator=(const router_forward_interface&);
	bool operator==(const router_forward_interface&) const;
};

inline router_forward_interface& router_forward_interface::operator =(const router_forward_interface &arg)
{
       ready=arg.ready;
	   vc_id=arg.vc_id;
	   data_flit=arg.data_flit;

       return (*this);
}

inline bool router_forward_interface::operator ==(const router_forward_interface &arg) const
{
	return (ready==arg.ready) && (vc_id==arg.vc_id) && (data_flit==arg.data_flit);
}


inline ostream&
operator<<(ostream& os, const router_forward_interface& arg)
{
	os<<"ready= "<<arg.ready<<", Vc_id="<<arg.vc_id<<",data:"<<arg.data_flit<<endl;
       return os;
}
extern  void sc_trace(sc_trace_file *tf, const router_forward_interface& arg, const  std::string& name) 
{
	sc_trace(tf,arg.vc_id,"src");

}

struct router_backward_interface
{
	bool buffer_full[vc_number];
	int available_vc;
	int vc_available[vc_number];//add for torus deadlock break
	
	router_backward_interface& operator=(const router_backward_interface&);
	bool operator==(const router_backward_interface&) const;
};

inline router_backward_interface& router_backward_interface::operator =(const router_backward_interface &arg)
{
	for(int i=0;i<vc_number;i++){
		   buffer_full[i]=arg.buffer_full[i];
		   vc_available[i]=arg.vc_available[i];
	}
	   available_vc=arg.available_vc;
       return (*this);
}

inline bool router_backward_interface::operator ==(const router_backward_interface &arg) const
{

	for(int i=0;i<vc_number;i++)
	{
		if(buffer_full[i]!=arg.buffer_full[i] || vc_available[i]!=arg.vc_available[i] )
			return false;
	}
	return (available_vc==arg.available_vc);
}



inline ostream&
operator<<(ostream& os, const router_backward_interface& arg)
{
	os<<"available_vc="<<arg.available_vc<<endl;
       return os;
}
extern  void sc_trace(sc_trace_file *tf, const router_backward_interface& arg, const  std::string& name) 
{
	sc_trace(tf,arg.available_vc,"src");

}

class interchip_control_packet_type
{
public:

	bool request;
	bool tear_down;
	bool grant;
	bool fail;
	int src_chip;
	int dest_chip;
	int bus_id;
	

	interchip_control_packet_type& operator = (const interchip_control_packet_type& arg)
	{
		request=arg.request;
		tear_down=arg.tear_down;
		grant=arg.grant;
		fail=arg.fail;
		src_chip=arg.src_chip;
		dest_chip=arg.dest_chip;
		bus_id=arg.bus_id;
		return (*this);
	}
	 bool operator==(const interchip_control_packet_type& arg) const
	 {
		 return ((request==arg.request)&&(tear_down==arg.tear_down)&&(grant==arg.grant)&&(fail==arg.fail)&&(src_chip==arg.src_chip)&&(dest_chip==arg.dest_chip)&&(bus_id==arg.bus_id));
	 }
};
inline ostream& 
operator<<(ostream& os, const interchip_control_packet_type& arg)
{
	os<<"req="<<arg.request<<endl;
       return os;
}

extern  void sc_trace(sc_trace_file *tf, const interchip_control_packet_type& arg, const  std::string& name) 
{
       sc_trace(tf,arg.request,"req");//,name ".src");
	   sc_trace(tf,arg.grant,"grant");//,name + ".dest");
	   sc_trace(tf,arg.fail,"fail");
}

//Generate_real_random_number_with U(0,1)
static double gen_uniform(){
	static bool gen_uniform_initial=false;
	if(!gen_uniform_initial)
	{
		srand((unsigned)time(NULL));
		gen_uniform_initial=true;
	}
	double temp=0;
	while(temp==0)
	{
		double tp= rand();
		if(tp<0 || tp>RAND_MAX)
		{
			cout<<"error generating rand()"<<endl;
		}
		temp= tp / ((double)RAND_MAX+1) ; //Notice you should not use RAND_MAX+1: overflow may happen
	}
	return temp;
}


// normal random variate generator
// mean m, standard deviation s
//Box-Muller Method to generate Gaussian
//http://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution
static double gen_normal_dist(double m, double s)
{
	double U=gen_uniform();
	double V=gen_uniform();
	double X=sqrt(-2*log(U))*cos(2*PI*V);
	return m+s*X;
}





#endif

