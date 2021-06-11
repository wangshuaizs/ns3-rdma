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

double cnp_interval = 50, alpha_resume_interval = 55, rp_timer, dctcp_gain = 1 / 16, np_sampling_interval = 0, pmax = 1;
uint32_t byte_counter, fast_recovery_times = 5, kmax = 60, kmin = 60;
std::string rate_ai, rate_hai;

bool clamp_target_rate = false, clamp_target_rate_after_timer = false, send_in_chunks = true, l2_wait_for_ack = false, l2_back_to_zero = false, l2_test_read = false;
double error_rate_per_link = 0.0;
NodeContainer n;
#define kkk 16
#define nnn 2  // nnn < 6
#define port_num 4000
uint32_t used_port[kkk*kkk] = { 0 };  //��1άΪpow(kkk,nnn),��2ά����Ϊnnn*nnn*(kkk-1)
int step[kkk*kkk] = { 0 };  //ά�ȴ�СΪpow(kkk,nnn)
uint64_t total_parameter = 640 * 1000; //640Mbytes
int pkt_num = total_parameter / (int)pow(kkk, nnn) / 2; //max = "4294967295";
int ring_order[kkk*kkk];  //ά�ȴ�СΪpow(kkk,nnn)
#define SCATTER_GATHER

std::string pcap_file = "rdma_bcube_ring_pcap";
#define FLOW_PATH        "mix/rdma_bcube_ring_flow.txt"
#define TOPO_PATH        "mix/bcube_topology.txt"
#define TRACE_PATH       "mix/rdma_bcube_ring_trace.txt"
#define RING_FILE        "mix/bcube_16_2_ring.txt"
#define TRACE_OUT_PATH   "mix/rdma_bcube_ring_256_640MB_200G_scattergather.tr"
int bcube_topology_generate(string path, int kk, int nn);
int ring_traffic(string path, int kk, int nn);
int trace_traffic(string path, int server_num);
int euler_circuit(int n);

ofstream topofile;
string topo_link_rate = "200Gbps";
string topo_link_delay = "0.001ms";
string topo_link_error_rate = "0";
ofstream flowfile;
string flow_priority = "3";
string flow_start_time = "1.0";
string flow_end_time = "5.0";

uint32_t iiii = 0;
void
Is_ready_to_send(Ptr<UdpServer> sink, uint32_t src, uint32_t dst)
{
	if (sink->GetReceived() >= pkt_num)
	{
		iiii++;
		if (iiii % (2 * (int)pow(kkk, nnn)) == 0)
		{
#ifdef SCATTER_GATHER
			cout << "finish steps / totoal steps : " << iiii / (2 * (int)pow(kkk, nnn)) << "/" << 2 * ((int)pow(kkk, nnn) - 1) << endl;
#else
			cout << "finish steps / totoal steps : " << iiii / (2 * (int)pow(kkk, nnn)) << "/" << (int)pow(kkk, nnn) - 1 << endl;
#endif // SCATTER_GATHER
		}

		bool no_more_schedule = true;
		step[dst]++;
#ifdef SCATTER_GATHER
		if (step[dst] < 2 * 2 * ((int)pow(kkk, nnn) - 1))
#else
		if (step[dst] < 2 * (int)pow(kkk, nnn) - 1)
#endif // SCATTER_GATHER
			no_more_schedule = false;

		uint32_t src_idx = -1, dst_idx = -1;
		for (int i = 0; i < (int)pow(kkk, nnn); i++)
		{
			if (ring_order[i] == src)
				src_idx = i;
			if (ring_order[i] == dst)
				dst_idx = i;
		}

		if (no_more_schedule == false)
		{
			// dir: 0:increase order; 1: decrease order
			int dir = src_idx < dst_idx ? 0 : 1;
			if (src_idx == 0 && dst_idx == (int)pow(kkk, nnn) - 1)
				dir = 1;
			else if (src_idx == (int)pow(kkk, nnn) - 1 && dst_idx == 0)
				dir = 0;

			src_idx = dst_idx;
			if (dir == 0)
			{
				if (src_idx != (int)pow(kkk, nnn) - 1)
				{
					dst_idx = src_idx + 1;
				}
				else {
					dst_idx = 0;
				}
			}
			else {
				if (src_idx != 0)
				{
					dst_idx = src_idx - 1;
				}
				else {
					dst_idx = (int)pow(kkk, nnn) - 1;
				}
			}
			src = ring_order[src_idx];
			dst = ring_order[dst_idx];

			uint32_t pg = 3, maxPacketCount = pkt_num, port;
			port = used_port[dst] + 1;
			used_port[dst]++;
			Ptr<Ipv4> ipv4 = n.Get(dst)->GetObject<Ipv4>();
			Ipv4Address serverAddress;
			int level;
			for (level = nnn - 1; level >= 0; level--)
				if (abs((int)(src - dst)) % (int)pow(kkk, level) == 0)
				{
					serverAddress = ipv4->GetAddress(level + 1, 0).GetLocal(); //GetAddress(0,0) is the loopback 127.0.0.1
					break;
				}

			UdpServerHelper server0(port);
			ApplicationContainer apps0s = server0.Install(n.Get(dst));

			UdpClientHelper client0(serverAddress, port, pg); //Add Priority
			client0.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
			client0.SetAttribute("Interval", TimeValue(Seconds(0.00000005 / 2)));
			client0.SetAttribute("PacketSize", UintegerValue(packet_payload_size));
			ApplicationContainer apps0c = client0.Install(n.Get(src));

			apps0s.Start(MicroSeconds(0.0));
			apps0c.Start(MicroSeconds(0.0));

			sink = server0.GetServer();
			Simulator::Schedule(MicroSeconds(1.0), &Is_ready_to_send, sink, src, dst);
			//cout << Simulator::Now().GetTimeStep() << "\t" << src << " -> " << dst << "\t" << sink->GetReceived() << "\t" << step[dst] << "\n";
		}
		else
		{
			bool fin = true;
			for (int j = 0; j < (int)pow(kkk, nnn); j++)
			{
#ifdef SCATTER_GATHER
				if (step[j] < 2 * 2 * ((int)pow(kkk, nnn) - 1))
#else
				if (step[j] < 2 * ((int)pow(kkk, nnn) - 1))
#endif // SCATTER_GATHER
				{
					fin = false;
					break;
				}
			}
			if (fin)
				cout << Simulator::Now().GetTimeStep() << "\t" << dst << "\tall done!" << endl;
		}
	}
	else
	{
		Simulator::Schedule(MicroSeconds(1.0), &Is_ready_to_send, sink, src, dst);
	}
}

int main(int argc, char *argv[])
{
	bcube_topology_generate(TOPO_PATH, kkk, nnn);
	ring_traffic(FLOW_PATH, kkk, nnn);
	trace_traffic(TRACE_PATH, (int)pow(kkk, nnn) + (int)pow(kkk, nnn - 1) * nnn);

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
	byte_counter = 300000000;
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
	uint32_t node_num, switch_num, link_num, flow_num, trace_num, tcp_flow_num;
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
	//qbb.EnablePcap(pcap_file, trace_nodes, true);

	Ipv4GlobalRoutingHelper::PopulateRoutingTables();
	std::cout << "Routing complete.\n";

	NS_LOG_INFO("Create Applications.");

	uint32_t packetSize = packet_payload_size;
	Time interPacketInterval = Seconds(0.00000005 / 2);

	for (uint32_t i = 0; i < flow_num; i++)
	{
		uint32_t src, dst, pg, maxPacketCount, port;
		double start_time, stop_time;
		flowf >> src >> dst >> pg >> maxPacketCount >> start_time >> stop_time;
		port = used_port[dst] + 1;
		used_port[dst]++;
		NS_ASSERT(n.Get(src)->GetNodeType() == 0 && n.Get(dst)->GetNodeType() == 0);
		Ptr<Ipv4> ipv4 = n.Get(dst)->GetObject<Ipv4>();
		Ipv4Address serverAddress;
		int level;
		for (level = nnn - 1; level >= 0; level--)
			if (abs((int)(src - dst)) % (int)pow(kkk, level) == 0)
			{
				serverAddress = ipv4->GetAddress(level + 1, 0).GetLocal(); //GetAddress(0,0) is the loopback 127.0.0.1
				break;
			}
		if (send_in_chunks)
		{
			UdpEchoServerHelper server0(port, pg); //Add Priority
			ApplicationContainer apps0s = server0.Install(n.Get(dst));
			apps0s.Start(Seconds(app_start_time));
			apps0s.Stop(Seconds(app_stop_time));
			UdpEchoClientHelper client0(serverAddress, port, pg); //Add Priority
			client0.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
			client0.SetAttribute("Interval", TimeValue(interPacketInterval));
			client0.SetAttribute("PacketSize", UintegerValue(packetSize));
			ApplicationContainer apps0c = client0.Install(n.Get(src));
			apps0c.Start(Seconds(start_time));
			apps0c.Stop(Seconds(stop_time));
		}
		else
		{
			UdpServerHelper server0(port);
			ApplicationContainer apps0s = server0.Install(n.Get(dst));

			UdpClientHelper client0(serverAddress, port, pg); //Add Priority
			client0.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
			client0.SetAttribute("Interval", TimeValue(interPacketInterval));
			client0.SetAttribute("PacketSize", UintegerValue(packetSize));
			ApplicationContainer apps0c = client0.Install(n.Get(src));

			apps0s.Start(Seconds(app_start_time));
			apps0c.Start(Seconds(start_time));

			Ptr<UdpServer> sink = server0.GetServer();
			Simulator::Schedule(Seconds(1.0), &Is_ready_to_send, sink, src, dst);
		}


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

void connect(int nn, int kk, int index_level, int offset, int sw_level)
{
	if (index_level > 0)
	{
		index_level--;
		for (int next_level_index = 0; next_level_index < kk; next_level_index++)
		{
			int extra_offset = next_level_index * (int)pow(kk, index_level);
			connect(nn, kk, index_level, offset + extra_offset, sw_level);
		}
	}
	else
	{
		int sw = (int)pow(kk, nn) + offset;
		int rest_pos[5];
		int ii = 0;
		for (int i = 0; i < nn; i++)
		{
			if (i == sw_level)
				rest_pos[i] = 0;
			else
			{
				rest_pos[i] = offset / (int)pow(kk, ii) % kk;
				ii++;
			}
		}
		int server_base = 0;
		for (int i = nnn - 1; i >= 0; i--)
			server_base = server_base*kk + rest_pos[i];
		for (int i = 0; i < kk; i++)
		{
			int server = server_base + i*(int)pow(kk, sw_level);
			topofile << sw << " " << server << " " << topo_link_rate << " " << topo_link_delay << " " << topo_link_error_rate << endl;
		}
	}
}

int bcube_topology_generate(string path, int kk, int nn)
{
	int server_num = (int)pow(kk, nn);
	int switch_num = (int)pow(kk, nn - 1) * nn;
	int total_node_num = server_num + switch_num;
	int link_num = (int)pow(kk, nn) * nn;

	topofile.open(path);
	// output first line, total node #, switch node #, link #
	topofile << total_node_num << " " << switch_num << " " << link_num << endl;
	// output seconde line, switch node IDs 
	for (int i = 0; i<switch_num; i++)
	{
		topofile << server_num + i << " ";
	}
	topofile << endl;

	//����ж�ÿ��������Ҫ����Щserver
	// output server to level 'j' switch links, src0 dst0 rate delay error_rate
	for (int i = 0; i < nn; i++)
	{
		int high_offset = i * (int)pow(kk, nn - 1);
		connect(nn, kk, nn - 1, high_offset, i);
	}

	// print description
	topofile << "\n\n";
	topofile << "First line : total node #, switch node #, link #" << endl;
	topofile << "Second line : switch node IDs..." << endl;
	topofile << "src0 dst0 rate delay error_rate" << endl;
	topofile << "src1 dst1 rate delay error_rate" << endl;
	topofile << "..." << endl;
	topofile << endl;
	topofile << "0 - " << server_num - 1 << " Servers" << endl;
	for (int i = 0; i < nn; i++)
		topofile << server_num + (int)pow(kk, nn - 1) * i << " - " << server_num + (int)pow(kk, nn - 1) * (i + 1) - 1 << " Level " << i << " switches" << endl;
	// close file
	topofile.close();

	return 0;
}

int ring_traffic(string path, int k, int nn)
{
	std::ifstream ringf;
	int flow_num = (int)pow(k, nn);
	uint32_t packet_num = pkt_num;

	flowfile.open(path);
	// output first line, flow #
	flowfile << 2 * flow_num << endl;

	euler_circuit(k);
	// output the rest line, src dst priority packet# start_time end_time
	for (int i = 0; i < (flow_num - 1); i++)
	{
		flowfile << ring_order[i] << " " << ring_order[i + 1] << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;
		flowfile << ring_order[i + 1] << " " << ring_order[i] << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;
	}
	flowfile << ring_order[flow_num - 1] << " " << ring_order[0] << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;
	flowfile << ring_order[0] << " " << ring_order[flow_num - 1] << " " << flow_priority << " " << pkt_num << " " << flow_start_time << " " << flow_end_time << endl;

	// print description
	flowfile << "\n\n";
	flowfile << "First line : flow#" << endl;
	flowfile << "src dst priority packet# start_time end_time" << endl;
	flowfile << "..." << endl;
	// close file
	flowfile.close();
	ringf.close();

	return 0;
}

int trace_traffic(string path, int server_num)
{
	ofstream tracefile;

	tracefile.open(path);
	//server_num = 0;
	tracefile << server_num << endl;
	for (int j = 0; j < server_num; j++)
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

int euler_circuit(int n)
{
	int servers = n*n; // only applied to k=2
	int cnt = 0;
	int eluer_circuit_path[kkk / 2 + 1][2 * kkk] = { 0 };
	// the first row of arrary 'eluer_circuit_path' is useless
	eluer_circuit_path[0][2 * n - 1] = 0;

	for (int i = 1; i < n / 2 + 1; i++)
		for (int j = 0; j < 2 * n; j++)
		{
			if (j == 0)
				eluer_circuit_path[i][j] = eluer_circuit_path[i - 1][2 * n - 1] % n;
			else if (j % 2 == 1)
			{
				int previous_neigh = eluer_circuit_path[i][j - 1];
				if ((previous_neigh + 1) % n != 0)
				{
					if (previous_neigh / n == n - 1)
					{
						eluer_circuit_path[i][j] = previous_neigh - 1;
					}
					else
					{
						eluer_circuit_path[i][j] = previous_neigh + 1;
					}
				}
				else
				{
					if (previous_neigh / n == n - 1)
					{
						eluer_circuit_path[i][j] = previous_neigh - 1;
					}
					else
					{
						eluer_circuit_path[i][j] = previous_neigh - n + 1;
					}
				}
			}
			else if (j % 2 == 0)
			{
				int previous_neigh = eluer_circuit_path[i][j - 1];
				if ((previous_neigh + n) < servers)
				{
					eluer_circuit_path[i][j] = previous_neigh + n;
				}
				else
				{
					eluer_circuit_path[i][j] = previous_neigh - 1;
				}
			}

		}

	int ri = 0;
	for (int i = 1; i < n / 2 + 1; i++)
	{
		for (int j = 0; j < 2 * n; j++)
		{
			int flag = true;
			for (int ii = 1; ii < n / 2 + 1; ii++)
				for (int jj = 0; jj < 2 * n; jj++)
					if (eluer_circuit_path[i][j] == eluer_circuit_path[ii][jj] && (i != ii && j != jj))
					{
						flag = false;
						break;
					}
			if (flag)
				ring_order[ri++] = eluer_circuit_path[i][j];
			cnt++;
		}
	}
	return 0;
}