/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include <iostream>
#include <fstream>
#include <time.h> 
#include <math.h>
#include "ns3/core-module.h"
#include "ns3/qbb-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/global-route-manager.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/broadcom-node.h"
#include "ns3/packet.h"
#include "ns3/error-model.h"
#include "ns3/netanim-module.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("GENERIC_SIMULATION");

// app_start_time and app_stop_time is for server
bool enable_qcn = true, use_dynamic_pfc_threshold = true, packet_level_ecmp = false, flow_level_ecmp = false;
uint32_t packet_payload_size = 1000, l2_chunk_size = 0, l2_ack_interval = 0;
double pause_time = 5, simulator_stop_time = 3.01, app_start_time = 1.0, app_stop_time = 9.0;
std::string data_rate, link_delay, topology_file, flow_file, trace_file, trace_output_file;

double cnp_interval = 50, alpha_resume_interval = 55, rp_timer, dctcp_gain = 1 / 256, np_sampling_interval = 0, pmax = 1;
uint32_t byte_counter, fast_recovery_times = 5, kmax = 60, kmin = 60;
std::string rate_ai, rate_hai;

bool clamp_target_rate = false, clamp_target_rate_after_timer = false, send_in_chunks = true, l2_wait_for_ack = false, l2_back_to_zero = false, l2_test_read = false;
double error_rate_per_link = 0.0;

NodeContainer n;
#define port_num 4000
bool used_port[100][port_num] = { false };
int pkt_num = 50 * 1000; // 50MBytes

#define FLOW_PATH         "mix/flow_fairness_flow.txt"
#define TOPO_PATH         "mix/topology.txt"
#define TRACE_PATH        "mix/flow_fairness_trace.txt"
#define TRACE_OUT_PATH    "mix/flow_fairness_mix.tr"
int topology_generate(string path);
int traffic(string path);
int trace_traffic(string path);

ofstream topofile;
string topo_link_rate = "40Gbps";
string topo_link_delay = "0.001ms";
string topo_link_error_rate = "0";
ofstream flowfile;
string flow_priority = "3";
string flow_start_time = "1.0";
string flow_end_time = "5.0";

Ptr<Ipv4StaticRouting> GetNodeStaticRoutingProtocol(Ptr<Node> node)
{
	Ptr<Ipv4> nodeIpv4 = node->GetObject<Ipv4>();
	Ptr<Ipv4RoutingProtocol> routingProtocol = nodeIpv4->GetRoutingProtocol();
	Ptr<Ipv4ListRouting> listRouting = DynamicCast<Ipv4ListRouting>(routingProtocol);
	int16_t priority;
	Ptr<Ipv4StaticRouting> routing = DynamicCast<Ipv4StaticRouting>(listRouting->GetRoutingProtocol(0, priority));
	return routing;
}

int main(int argc, char *argv[])
{
	topology_generate(TOPO_PATH);
	traffic(FLOW_PATH);
	trace_traffic(TRACE_PATH);

	topology_file = TOPO_PATH;
	flow_file = FLOW_PATH;
	trace_file = TRACE_PATH;
	trace_output_file = TRACE_OUT_PATH;
	app_start_time = 0.99;
	app_stop_time = 8.01;
	simulator_stop_time = 10.0;
	packet_payload_size = 1000;
	send_in_chunks = 0;
	enable_qcn = 1;
	use_dynamic_pfc_threshold = 1;
	packet_level_ecmp = 0;
	flow_level_ecmp = 1;
	clamp_target_rate = 1;
	clamp_target_rate_after_timer = 0;
	pause_time = 5;
	l2_chunk_size = 4000;
	l2_ack_interval = 256;
	l2_wait_for_ack = 0;
	l2_back_to_zero = 0;
	l2_test_read = false;
	cnp_interval = 50;
	alpha_resume_interval = 55;
	rp_timer = 60;
	byte_counter = 10000000;
	kmax = 1000;
	kmin = 40;
	pmax = 0.01;
	dctcp_gain = 0.00390625;
	fast_recovery_times = 5;
	rate_ai = "40Mb/s";
	rate_hai = "200Mb/s";
	np_sampling_interval = 0;
	error_rate_per_link = 0.0;


	std::cout << "TOPOLOGY_FILE\t\t\t" << topology_file << "\n";
	std::cout << "FLOW_FILE\t\t\t" << flow_file << "\n";
	std::cout << "TRACE_FILE\t\t\t" << trace_file << "\n";
	std::cout << "TRACE_OUTPUT_FILE\t\t" << trace_output_file << "\n";
	std::cout << "SINK_START_TIME\t\t\t" << app_start_time << "\n";
	std::cout << "SINK_STOP_TIME\t\t\t" << app_stop_time << "\n";
	std::cout << "SIMULATOR_STOP_TIME\t\t" << simulator_stop_time << "\n";
	std::cout << "PACKET_PAYLOAD_SIZE\t\t" << packet_payload_size << "\n";
	if (send_in_chunks)
	{
		std::cout << "SEND_IN_CHUNKS\t\t\t" << "Yes" << "\n";
		std::cout << "WARNING: deprecated and not tested. Please consider using L2_WAIT_FOR_ACK";
	}
	else
		std::cout << "SEND_IN_CHUNKS\t\t\t" << "No" << "\n";
	if (enable_qcn)
		std::cout << "ENABLE_QCN\t\t\t" << "Yes" << "\n";
	else
		std::cout << "ENABLE_QCN\t\t\t" << "No" << "\n";
	if (use_dynamic_pfc_threshold)
		std::cout << "USE_DYNAMIC_PFC_THRESHOLD\t" << "Yes" << "\n";
	else
		std::cout << "USE_DYNAMIC_PFC_THRESHOLD\t" << "No" << "\n";
	if (packet_level_ecmp)
		std::cout << "PACKET_LEVEL_ECMP\t\t" << "Yes" << "\n";
	else
		std::cout << "PACKET_LEVEL_ECMP\t\t" << "No" << "\n";
	if (flow_level_ecmp)
		std::cout << "FLOW_LEVEL_ECMP\t\t\t" << "Yes" << "\n";
	else
		std::cout << "FLOW_LEVEL_ECMP\t\t\t" << "No" << "\n";
	if (clamp_target_rate)
		std::cout << "CLAMP_TARGET_RATE\t\t" << "Yes" << "\n";
	else
		std::cout << "CLAMP_TARGET_RATE\t\t" << "No" << "\n";
	if (clamp_target_rate_after_timer)
		std::cout << "CLAMP_TARGET_RATE_AFTER_TIMER\t" << "Yes" << "\n";
	else
		std::cout << "CLAMP_TARGET_RATE_AFTER_TIMER\t" << "No" << "\n";
	std::cout << "PAUSE_TIME\t\t\t" << pause_time << "\n";
	std::cout << "L2_CHUNK_SIZE\t\t\t" << l2_chunk_size << "\n";
	std::cout << "L2_ACK_INTERVAL\t\t\t" << l2_ack_interval << "\n";
	if (l2_wait_for_ack)
		std::cout << "L2_WAIT_FOR_ACK\t\t\t" << "Yes" << "\n";
	else
		std::cout << "L2_WAIT_FOR_ACK\t\t\t" << "No" << "\n";
	if (l2_back_to_zero)
		std::cout << "L2_BACK_TO_ZERO\t\t\t" << "Yes" << "\n";
	else
		std::cout << "L2_BACK_TO_ZERO\t\t\t" << "No" << "\n";
	if (l2_test_read)
		std::cout << "L2_TEST_READ\t\t\t" << "Yes" << "\n";
	else
		std::cout << "L2_TEST_READ\t\t\t" << "No" << "\n";
	std::cout << "CNP_INTERVAL\t\t\t" << cnp_interval << "\n";
	std::cout << "ALPHA_RESUME_INTERVAL\t\t" << alpha_resume_interval << "\n";
	std::cout << "RP_TIMER\t\t\t" << rp_timer << "\n";
	std::cout << "BYTE_COUNTER\t\t\t" << byte_counter << "\n";
	std::cout << "KMAX\t\t\t\t" << kmax << "\n";
	std::cout << "PMAX\t\t\t\t" << pmax << "\n";
	std::cout << "KMIN\t\t\t\t" << kmin << "\n";
	std::cout << "DCTCP_GAIN\t\t\t" << dctcp_gain << "\n";
	std::cout << "FAST_RECOVERY_TIMES\t\t" << fast_recovery_times << "\n";
	std::cout << "RATE_AI\t\t\t\t" << rate_ai << "\n";
	std::cout << "RATE_HAI\t\t\t" << rate_hai << "\n";
	std::cout << "NP_SAMPLING_INTERVAL\t\t" << np_sampling_interval << "\n";
	std::cout << "ERROR_RATE_PER_LINK\t\t" << error_rate_per_link << "\n";
	fflush(stdout);

	bool dynamicth = use_dynamic_pfc_threshold;

	NS_ASSERT(packet_level_ecmp + flow_level_ecmp < 2); //packet level ecmp and flow level ecmp are exclusive
	Config::SetDefault("ns3::Ipv4GlobalRouting::RandomEcmpRouting", BooleanValue(packet_level_ecmp));
	Config::SetDefault("ns3::Ipv4GlobalRouting::FlowEcmpRouting", BooleanValue(flow_level_ecmp));
	Config::SetDefault("ns3::QbbNetDevice::PauseTime", UintegerValue(pause_time));
	Config::SetDefault("ns3::QbbNetDevice::QcnEnabled", BooleanValue(enable_qcn));
	Config::SetDefault("ns3::QbbNetDevice::DynamicThreshold", BooleanValue(dynamicth));
	Config::SetDefault("ns3::QbbNetDevice::ClampTargetRate", BooleanValue(clamp_target_rate));
	Config::SetDefault("ns3::QbbNetDevice::ClampTargetRateAfterTimeInc", BooleanValue(clamp_target_rate_after_timer));
	Config::SetDefault("ns3::QbbNetDevice::CNPInterval", DoubleValue(cnp_interval));
	Config::SetDefault("ns3::QbbNetDevice::NPSamplingInterval", DoubleValue(np_sampling_interval));
	Config::SetDefault("ns3::QbbNetDevice::AlphaResumInterval", DoubleValue(alpha_resume_interval));
	Config::SetDefault("ns3::QbbNetDevice::RPTimer", DoubleValue(rp_timer));
	Config::SetDefault("ns3::QbbNetDevice::ByteCounter", UintegerValue(byte_counter));
	Config::SetDefault("ns3::QbbNetDevice::FastRecoveryTimes", UintegerValue(fast_recovery_times));
	Config::SetDefault("ns3::QbbNetDevice::DCTCPGain", DoubleValue(dctcp_gain));
	Config::SetDefault("ns3::QbbNetDevice::RateAI", DataRateValue(DataRate(rate_ai)));
	Config::SetDefault("ns3::QbbNetDevice::RateHAI", DataRateValue(DataRate(rate_hai)));
	Config::SetDefault("ns3::QbbNetDevice::L2BackToZero", BooleanValue(l2_back_to_zero));
	Config::SetDefault("ns3::QbbNetDevice::L2TestRead", BooleanValue(l2_test_read));
	Config::SetDefault("ns3::QbbNetDevice::L2ChunkSize", UintegerValue(l2_chunk_size));
	Config::SetDefault("ns3::QbbNetDevice::L2AckInterval", UintegerValue(l2_ack_interval));
	Config::SetDefault("ns3::QbbNetDevice::L2WaitForAck", BooleanValue(l2_wait_for_ack));

	clock_t begint, endt;
	begint = clock();

	SeedManager::SetSeed(time(NULL));

	std::ifstream topof, flowf, tracef;
	topof.open(topology_file.c_str());
	flowf.open(flow_file.c_str());
	tracef.open(trace_file.c_str());
	uint32_t node_num, switch_num, link_num, flow_num, trace_num;
	topof >> node_num >> switch_num >> link_num;
	flowf >> flow_num;
	tracef >> trace_num;


	n.Create(node_num);
	for (uint32_t i = 0; i < switch_num; i++)
	{
		uint32_t sid;
		topof >> sid;
		n.Get(sid)->SetNodeType(1, dynamicth); //broadcom switch
		n.Get(sid)->m_broadcom->SetMarkingThreshold(kmin, kmax, pmax);
	}
	NS_LOG_INFO("Create nodes.");

	InternetStackHelper internet;
	internet.Install(n);
	NS_LOG_INFO("Create channels.");

	//
	// Explicitly create the channels required by the topology.
	//
	Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
	Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
	rem->SetRandomVariable(uv);
	uv->SetStream(50);
	rem->SetAttribute("ErrorRate", DoubleValue(error_rate_per_link));
	rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));

	QbbHelper qbb;
	Ipv4AddressHelper ipv4;
	for (uint32_t i = 0; i < link_num; i++)
	{
		uint32_t src, dst;
		std::string data_rate, link_delay;
		double error_rate;
		topof >> src >> dst >> data_rate >> link_delay >> error_rate;

		qbb.SetDeviceAttribute("DataRate", StringValue(data_rate));
		qbb.SetChannelAttribute("Delay", StringValue(link_delay));

		if (error_rate > 0)
		{
			Ptr<RateErrorModel> rem = CreateObject<RateErrorModel>();
			Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
			rem->SetRandomVariable(uv);
			uv->SetStream(50);
			rem->SetAttribute("ErrorRate", DoubleValue(error_rate));
			rem->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
			qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
		}
		else
		{
			qbb.SetDeviceAttribute("ReceiveErrorModel", PointerValue(rem));
		}

		fflush(stdout);
		NetDeviceContainer d = qbb.Install(n.Get(src), n.Get(dst));

		char ipstring[16];
		sprintf(ipstring, "10.%d.%d.0", i / 254 + 1, i % 254 + 1);
		ipv4.SetBase(ipstring, "255.255.255.0");
		ipv4.Assign(d);
	}


	NodeContainer trace_nodes;
	for (uint32_t i = 0; i < trace_num; i++)
	{
		uint32_t nid;
		tracef >> nid;
		trace_nodes = NodeContainer(trace_nodes, n.Get(nid));
	}
	AsciiTraceHelper ascii;
	qbb.EnableAscii(ascii.CreateFileStream(trace_output_file), trace_nodes);

	Ptr<Ipv4StaticRouting> routing;
	routing = GetNodeStaticRoutingProtocol(n.Get(5));
	routing->AddHostRouteTo(Ipv4Address("10.1.14.2"), Ipv4Address("10.1.6.2"), 6);
	routing->AddHostRouteTo(Ipv4Address("10.1.15.2"), Ipv4Address("10.1.6.2"), 6);
	routing->AddHostRouteTo(Ipv4Address("10.1.16.2"), Ipv4Address("10.1.7.2"), 7);
	routing->AddHostRouteTo(Ipv4Address("10.1.17.2"), Ipv4Address("10.1.7.2"), 7);
	routing->AddHostRouteTo(Ipv4Address("10.1.18.2"), Ipv4Address("10.1.7.2"), 7);

	routing = GetNodeStaticRoutingProtocol(n.Get(6));
	routing->AddHostRouteTo(Ipv4Address("10.1.14.2"), Ipv4Address("10.1.8.2"), 2);
	routing->AddHostRouteTo(Ipv4Address("10.1.15.2"), Ipv4Address("10.1.8.2"), 2);

	routing = GetNodeStaticRoutingProtocol(n.Get(7));
	routing->AddHostRouteTo(Ipv4Address("10.1.16.2"), Ipv4Address("10.1.10.2"), 2);
	routing->AddHostRouteTo(Ipv4Address("10.1.17.2"), Ipv4Address("10.1.11.2"), 3);
	routing->AddHostRouteTo(Ipv4Address("10.1.18.2"), Ipv4Address("10.1.11.2"), 3);

	Ipv4GlobalRoutingHelper::PopulateRoutingTables();
	std::cout << "Routing complete.\n";

	NS_LOG_INFO("Create Applications.");

	uint32_t packetSize = packet_payload_size;
	Time interPacketInterval = Seconds(0.0000005 / 2);

	for (uint32_t i = 0; i < flow_num; i++)
	{
		uint32_t src, dst, pg, maxPacketCount, port;
		double start_time, stop_time;
		flowf >> src >> dst >> pg >> maxPacketCount >> start_time >> stop_time;
		while (used_port[dst][port = int(UniformVariable(0, 1).GetValue() * (port_num - 1)) + 1])
			continue;
		used_port[dst][port] = true;
		NS_ASSERT(n.Get(src)->GetNodeType() == 0 && n.Get(dst)->GetNodeType() == 0);
		Ptr<Ipv4> ipv4 = n.Get(dst)->GetObject<Ipv4>();
		Ipv4Address serverAddress = ipv4->GetAddress(1, 0).GetLocal(); //GetAddress(0,0) is the loopback 127.0.0.1

		UdpServerHelper server0(port);
		ApplicationContainer apps0s = server0.Install(n.Get(dst));

		UdpClientHelper client0(serverAddress, port, pg); //Add Priority
		client0.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
		client0.SetAttribute("Interval", TimeValue(interPacketInterval));
		client0.SetAttribute("PacketSize", UintegerValue(packetSize));
		ApplicationContainer apps0c = client0.Install(n.Get(src));

		apps0s.Start(Seconds(app_start_time));
		apps0c.Start(Seconds(start_time));
	}

	std::cout << "Generate traffic complete.\n";

	topof.close();
	flowf.close();
	tracef.close();

	//
	// Now, do the actual simulation.
	//
	std::cout << "Running Simulation.\n";
	fflush(stdout);
	NS_LOG_INFO("Run Simulation.");
	Simulator::Stop(Seconds(simulator_stop_time));
	Simulator::Run();
	Simulator::Destroy();
	std::cout << "Done.\n";
	NS_LOG_INFO("Done.");

	endt = clock();
	std::cout << (double)(endt - begint) / CLOCKS_PER_SEC << " s\n";
}

int topology_generate(string path)
{
	topofile.open(path);
	// output first line, total node #, switch node #, link #
	topofile << 16 << " " << 6 << " " << 18 << endl;
	topofile << "5 6 7 8 9 10" << endl;
	// output links, src0 dst0 rate delay error_rate
	topofile << 0 << " " << 5 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 1 << " " << 5 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 2 << " " << 5 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 3 << " " << 5 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 4 << " " << 5 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 5 << " " << 6 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 5 << " " << 7 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 6 << " " << 8 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 6 << " " << 9 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 7 << " " << 8 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 7 << " " << 9 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 8 << " " << 10 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 9 << " " << 10 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 10 << " " << 11 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 10 << " " << 12 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 10 << " " << 13 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 10 << " " << 14 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	topofile << 10 << " " << 15 << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
	// print description
	topofile << "\n\n";
	topofile << "First line : total node #, switch node #, link #" << endl;
	topofile << "src0 dst0 rate delay error_rate" << endl;
	topofile << "src1 dst1 rate delay error_rate" << endl;
	topofile << "..." << endl;
	// close file
	topofile.close();

	return 0;
}

int traffic(string path)
{
	flowfile.open(path);
	// output first line, flow #
	flowfile << 5 << endl;
	// output the rest line, src dst priority packet# start_time end_time
	flowfile << 0 << " " << 11 << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;
	flowfile << 1 << " " << 12 << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;
	flowfile << 2 << " " << 13 << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;
	flowfile << 3 << " " << 14 << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;
	flowfile << 4 << " " << 15 << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;

	// print description
	flowfile << "\n\n";
	flowfile << "First line : flow#" << endl;
	flowfile << "src dst priority packet# start_time end_time" << endl;
	flowfile << "..." << endl;
	// close file
	flowfile.close();

	return 0;
}

int trace_traffic(string path)
{
	ofstream tracefile;

	tracefile.open(path);
	tracefile << 16 << endl;
	for (int j = 0; j < 16; j++)
	{
		tracefile << j << endl;
	}

	tracefile << "\n\nFirst line: tracing node #" << endl;
	tracefile << "Node IDs..." << endl;
	tracefile << "timestamp, node_being_traced, src_ip>dst_ip, u=udp, port#, sequence#, priority" << endl;
	// close file
	tracefile.close();

	return 0;
}