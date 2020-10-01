/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
TOPOLOGY_FILE mix/fat-tree-topology.txt
FLOW_FILE mix/flow.txt
TCP_FLOW_FILE mix/flow_tcp_0.txt
TRACE_FILE mix/trace.txt
TRACE_OUTPUT_FILE mix/mix.tr

APP_START_TIME 0.99
APP_STOP_TIME 8.01
SIMULATOR_STOP_TIME 10.0
PACKET_PAYLOAD_SIZE 1024
SEND_IN_CHUNKS 0

USE_DYNAMIC_PFC_THRESHOLD 1
PAUSE_TIME 5

PACKET_LEVEL_ECMP 1
FLOW_LEVEL_ECMP 0

ENABLE_QCN 1

KMAX 100000
KMIN 5000
PMAX 0.1

NP_SAMPLING_INTERVAL 0
CNP_INTERVAL 50

CLAMP_TARGET_RATE 1
CLAMP_TARGET_RATE_AFTER_TIMER 0
DCTCP_GAIN 0.00390625
ALPHA_RESUME_INTERVAL 55
RP_TIMER 60
BYTE_COUNTER 300000000
FAST_RECOVERY_TIMES 5

RATE_AI 40Mb/s
RATE_HAI 200Mb/s

L2_WAIT_FOR_ACK 0
L2_ACK_INTERVAL 256
L2_CHUNK_SIZE 4000
L2_BACK_TO_ZERO 0

ERROR_RATE_PER_LINK 0.0000
*/


#include <iostream>
#include <fstream>
#include <time.h> 
#include <vector>
#include <math.h>
#include <algorithm> // std::move_backward
#include <random> // std::default_random_engine
#include <chrono> // std::chrono::system_clock
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
uint32_t packet_payload_size = 1024, l2_chunk_size = 0, l2_ack_interval = 0;
double pause_time = 5, simulator_stop_time = 3.01, app_start_time = 1.0, app_stop_time = 9.0;
std::string data_rate, link_delay, topology_file, flow_file, tcp_flow_file, trace_file, trace_output_file;

double cnp_interval = 50, alpha_resume_interval = 55, rp_timer, dctcp_gain = 1 / 16, np_sampling_interval = 0, pmax = 1;
uint32_t byte_counter, fast_recovery_times = 5, kmax = 60, kmin = 60;
std::string rate_ai, rate_hai;

bool clamp_target_rate = false, clamp_target_rate_after_timer = false, send_in_chunks = true, l2_wait_for_ack = false, l2_back_to_zero = false, l2_test_read = false;
double error_rate_per_link = 0.0;
NodeContainer n;
#define SERVER_NUM 3
#define LARYER_NUM 19
#define PRIORITY_NUM 8
#define USED_PRIORITY_NUM 2
#define USED_HIGHEST_PRIORITY 3
uint32_t pkt_num = 4294967295; // 18717; //max = "4294967295";

						  // vector<uint32_t> layer_paras : unit Bytes
						  // vector<uint32_t> op_times : unit us
						  // VGG19
vector<uint32_t> layer_paras = { 7168, 147712, 295424, 590336, 1180672, 2360320, 2360320, 2360320, 4720640, 9439232, 9439232, 9439232, 9439232, 9439232, 9439232, 9439232, 411058176, 67125248, 16404388 };
uint32_t op_times[] = { 30319, 91665, 39119, 53470, 26756, 42353, 42394, 42367, 24336, 39367, 39365, 39363, 13403, 13374, 13378, 13374, 10459, 1634, 634 };

uint32_t para_sizes[LARYER_NUM];
uint32_t global_recv_send_index_order[SERVER_NUM][SERVER_NUM][LARYER_NUM]; // recv/sender/para
uint32_t recv_send_index_order[SERVER_NUM][SERVER_NUM][PRIORITY_NUM][LARYER_NUM + 1]; // recv/sender/pg/para
vector<uint32_t> ready_patitions(SERVER_NUM);

std::string pcap_file = "mix/rdma_one_switch_one2one_pcap";
std::string FLOW_PATH = "mix/rdma_one_switch_one2one_flow.txt";
std::string TOPO_PATH = "mix/one_switch-topology.txt";
std::string TRACE_PATH = "mix/rdma_one_switch_one2one_trace.txt";
std::string order_filepath = "mix/order_file.txt";
std::string MIX_PATH = "mix/rdma_one_switch_one2one_mix.tr";
int one_switch_topology_generate(string path, int k);
int one2one_traffic(string path, int server_num);
int tracetraffice(string path, int server_num);

void generate_global_random_send_order(void)
{
	ofstream orderfile;
	orderfile.open(order_filepath);
	vector<int> indexs;
	int para_count = 0;
	for (auto& it : layer_paras) {
		para_sizes[para_count] = ceil(it*1.0 / SERVER_NUM/10);
		//std::cout << para_sizes[para_count] << " ";
		indexs.push_back(para_count++);
	}
	//std::cout << "\n";

	for (int i = 0; i < SERVER_NUM; i++) {
		for (int j = 0; j < SERVER_NUM; j++) {
			//unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
			//shuffle(indexs.begin(), indexs.end(), std::default_random_engine(seed));
			unsigned seed = (i + 3)*(j + 7);
			shuffle(indexs.begin(), indexs.end(), std::default_random_engine(seed));

			int k = 0;
			for (auto& it : indexs) {
				global_recv_send_index_order[i][j][k] = it;
				k++;
			}
			/*for (int kk = 0; kk < LARYER_NUM; kk++) {
			std::cout << global_recv_send_index_order[i][j][kk] << " ";
			}
			std::cout << "\n";*/
			for (int layer_i = 0; layer_i < LARYER_NUM; layer_i++)
				orderfile << global_recv_send_index_order[i][j][layer_i] << " ";
			orderfile << "\n";
		}
	}

	orderfile.close();
}

void read_random_send_order(void)
{
	ifstream orderfile;
	ofstream priorityorderfile;
	orderfile.open(order_filepath.c_str());
	priorityorderfile.open("mix/priority_order_file.txt");

	for (int i = 0; i < SERVER_NUM; i++) {
		for (int j = 0; j < SERVER_NUM; j++) {
			for (int layer_i = 0; layer_i < LARYER_NUM; layer_i++)
				orderfile >> global_recv_send_index_order[i][j][layer_i];
		}
	}

#if (USED_PRIORITY_NUM > 1)
	uint32_t priority_thresholds[SERVER_NUM][USED_PRIORITY_NUM - 1];
	for (int i = 0; i < SERVER_NUM; i++)
		for (int j = 0; j < USED_PRIORITY_NUM - 1; j++)
			priority_thresholds[i][j] = 8;
	for (int i = 0; i < SERVER_NUM; i++)
		for (int j = 0; j < SERVER_NUM; j++) {
			recv_send_index_order[i][j][USED_HIGHEST_PRIORITY][0] = priority_thresholds[j][0] + 1;
			for (int prio_i = 1; prio_i < USED_PRIORITY_NUM - 1; prio_i++) {
				recv_send_index_order[i][j][USED_HIGHEST_PRIORITY - prio_i][0] = priority_thresholds[j][prio_i] - priority_thresholds[j][prio_i - 1];
			}
			recv_send_index_order[i][j][USED_HIGHEST_PRIORITY - USED_PRIORITY_NUM + 1][0] = LARYER_NUM - priority_thresholds[j][USED_PRIORITY_NUM - 2] - 1;

			for (int prio_i = 0; prio_i < USED_PRIORITY_NUM; prio_i++) {
				uint32_t start_from = prio_i == 0 ? 0 : priority_thresholds[j][prio_i - 1] + 1;
				uint32_t end_to = prio_i == (USED_PRIORITY_NUM - 1) ? LARYER_NUM - 1 : priority_thresholds[j][prio_i];
				int para_count = 1;
				for (int layer_i = 0; layer_i < LARYER_NUM; layer_i++) {
					if (global_recv_send_index_order[i][j][layer_i] >= start_from && global_recv_send_index_order[i][j][layer_i] <= end_to) {
						recv_send_index_order[i][j][USED_HIGHEST_PRIORITY - prio_i][para_count++] = global_recv_send_index_order[i][j][layer_i];
					}
				}
			}
			for (int prio_i = 0; prio_i < USED_PRIORITY_NUM; prio_i++) {
				for (int layer_i = 0; layer_i < LARYER_NUM + 1; layer_i++)
					priorityorderfile << recv_send_index_order[i][j][USED_HIGHEST_PRIORITY - USED_PRIORITY_NUM + 1 + prio_i][layer_i] << " ";
				priorityorderfile << "\n";
			}
		}
#else
	for (int i = 0; i < SERVER_NUM; i++)
		for (int j = 0; j < SERVER_NUM; j++) {
			recv_send_index_order[i][j][USED_HIGHEST_PRIORITY][0] = LARYER_NUM;
			for (int layer_i = 0; layer_i < LARYER_NUM; layer_i++)
				recv_send_index_order[i][j][USED_HIGHEST_PRIORITY][layer_i + 1] = global_recv_send_index_order[i][j][layer_i];

			for (int layer_i = 0; layer_i < LARYER_NUM + 1; layer_i++)
				priorityorderfile << recv_send_index_order[i][j][USED_HIGHEST_PRIORITY][layer_i] << " ";
			priorityorderfile << "\n";
		}
#endif
	orderfile.close();
	priorityorderfile.close();
}

int main(int argc, char *argv[])
{
	one_switch_topology_generate(TOPO_PATH, SERVER_NUM);
	one2one_traffic(FLOW_PATH, SERVER_NUM);
	tracetraffice(TRACE_PATH, SERVER_NUM);
	generate_global_random_send_order();
	read_random_send_order();

	topology_file = TOPO_PATH;
	flow_file = FLOW_PATH;
	trace_file = TRACE_PATH;
	trace_output_file = MIX_PATH;
	app_start_time = 0.0;
	app_stop_time = 10.0;
	simulator_stop_time = 10.0;
	send_in_chunks = 0;
	enable_qcn = 0;
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
	kmax = 100000;
	kmin = 5000;
	pmax = 0.1;
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

	//SeedManager::SetSeed(time(NULL));

	std::ifstream topof, flowf, tracef, tcpflowf;
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
	NodeContainer trace_nodes2;
	for (uint32_t i = 0; i < 3; i++)
	{
		trace_nodes2 = NodeContainer(trace_nodes2, n.Get(i));
	}
	AsciiTraceHelper ascii;
	qbb.EnableAscii(ascii.CreateFileStream(trace_output_file), trace_nodes);
	qbb.EnablePcap(pcap_file, trace_nodes2, true);

	Ipv4GlobalRoutingHelper::PopulateRoutingTables();
	std::cout << "Routing complete.\n";

	NS_LOG_INFO("Create Applications.");

	uint32_t packetSize = packet_payload_size;
	Time interPacketInterval = Seconds(0.0000005 / 2);

	for (uint32_t i = 0; i < SERVER_NUM; i++)
	{
		WorkerHelper worker0(5001);
		worker0.SetAttribute("WorkerID", UintegerValue(i));
		worker0.SetAttribute("PacketSize", UintegerValue(packetSize));
		worker0.SetAttribute("NumLayers", UintegerValue(LARYER_NUM));
		worker0.SetAttribute("NumServers", UintegerValue(SERVER_NUM));
		worker0.SetAttribute("ParameterSizes", UintegerValue((uint64_t)para_sizes));
		worker0.SetAttribute("OperatorTimes", UintegerValue((uint64_t)op_times));
		//UdpServerHelper worker0(port);
		ApplicationContainer apps0s = worker0.Install(n.Get(i));
		apps0s.Start(Seconds(app_start_time));
	}
	for (uint32_t i = 0; i < flow_num; i++)
	{
		uint32_t src, dst, pg, maxPacketCount, port;
		double start_time, stop_time;
		flowf >> src >> dst >> pg >> maxPacketCount >> start_time >> stop_time;
		NS_ASSERT(n.Get(src)->GetNodeType() == 0 && n.Get(dst)->GetNodeType() == 0);
		Ptr<Ipv4> ipv4 = n.Get(dst)->GetObject<Ipv4>();
		Ipv4Address serverAddress = ipv4->GetAddress(1, 0).GetLocal(); //GetAddress(0,0) is the loopback 127.0.0.1

		PSHelper ps0(serverAddress, 5001, pg, (uint64_t)recv_send_index_order, (uint64_t)para_sizes); //Add Priority
		//ps0.SetAttribute("PSID", UintegerValue(dst));
		//ps0.SetAttribute("IndexOrder", UintegerValue((uint64_t)&recv_send_index_order));
		//UdpClientHelper ps0(serverAddress, port, pg); //Add Priority
		ps0.SetAttribute("MaxPackets", UintegerValue(maxPacketCount));
		ps0.SetAttribute("Interval", TimeValue(interPacketInterval));
		ps0.SetAttribute("PacketSize", UintegerValue(packetSize));
		ps0.SetAttribute("PSID", UintegerValue(src));
		ps0.SetAttribute("ToWorker", UintegerValue(dst));
		ps0.SetAttribute("NumLayers", UintegerValue(LARYER_NUM));
		ps0.SetAttribute("NumServers", UintegerValue(SERVER_NUM));
		ps0.SetAttribute("NumPriorities", UintegerValue(USED_PRIORITY_NUM));
		ApplicationContainer apps0c = ps0.Install(n.Get(src));

		apps0c.Start(Seconds(start_time));

		//Ptr<UdpServer> sink = worker0.GetServer();
		//Simulator::Schedule(Seconds(0.0), &stream2parameter, sink, src, dst);


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

	std::ofstream endfile;
	endfile.open("end.txt");
	endfile << (double)(endt - begint) / CLOCKS_PER_SEC << " s\n";
	endfile.close();

	//char pause;
	//cin >> pause;
}

int one_switch_topology_generate(string path, int k)
{
	ofstream topofile;
	string link_rate = "25Gbps";
	string link_delay = "0.001ms";
	string link_error_rate = "0";
	int server_num = SERVER_NUM;
	int switch_num = 1;
	int total_node_num = server_num + switch_num;
	int link_num = server_num;

	topofile.open(path);
	// output first line, total node #, switch node #, link #
	topofile << total_node_num << " " << switch_num << " " << link_num << endl;
	// output seconde line, switch node IDs 
	for (int i = 0; i<switch_num; i++)
	{
		topofile << server_num + i << " ";
	}
	topofile << endl;
	// output server to edge switch links, src0 dst0 rate delay error_rate
	for (int i = 0; i < server_num; i++)
	{
		topofile << i << " " << server_num << " " << link_rate << " " << link_delay << " " << link_error_rate << endl;
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
	topofile << server_num << " - " << (server_num + k * k / 2 - 1) << " Edge switches" << endl;
	topofile << (server_num + k * k / 2) << " - " << (server_num + k * k - 1) << " Aggregation switches" << endl;
	topofile << (server_num + k * k) << " - " << (server_num + k * k + (k * k / 4) - 1) << " Core switches" << endl;
	// close file
	topofile.close();

	return 0;
}

int one2one_traffic(string path, int server_num)
{
	ofstream flowfile;
	int flow_num = server_num * (server_num - 1);
	string priority = "2";
	uint32_t packet_num = pkt_num; //max = "4294967295";
	string start_time = "0.0";
	string end_time = "10.0";

	flowfile.open(path);
	// output first line, flow #
	flowfile << USED_PRIORITY_NUM *flow_num << endl;
	for (int i = 0; i < server_num; i++)
		for (int j = 0; j < server_num; j++)
			if (i != j) {
				for (int k = 0; k < USED_PRIORITY_NUM; k++)
					flowfile << i << " " << j << " " << std::to_string((USED_HIGHEST_PRIORITY - k)) << " " << packet_num << " " << start_time << " " << end_time << std::endl;
				//flowfile << i << " " << j << " " << "3" << " " << packet_num << " " << start_time << " " << end_time << std::endl;
			}
	//flowfile << 2 << endl;
	// output the rest line, src dst priority packet# start_time end_time
	//flowfile << 1 << " " << 0 << " " << priority << " " << packet_num << " " << start_time << " " << end_time << endl;
	//flowfile << 1 << " " << 0 << " " << "4" << " " << packet_num << " " << start_time << " " << end_time << endl;
	// print description
	flowfile << "\n\n";
	flowfile << "First line : flow#" << endl;
	flowfile << "src dst priority packet# start_time end_time" << endl;
	flowfile << "..." << endl;
	// close file
	flowfile.close();

	return 0;
}

int tracetraffice(string path, int server_num)
{
	ofstream tracefile;

	tracefile.open(path);
	tracefile << server_num << endl;
	for (int i = 0; i < server_num; i++)
		tracefile << i << endl;
	/*tracefile << 3 << endl;
	tracefile << 0 << endl;
	tracefile << 1 << endl;
	tracefile << 2 << endl;*/

	tracefile << "\n\nFirst line: tracing node #" << endl;
	tracefile << "Node IDs..." << endl;
	tracefile << "timestamp, node_being_traced, src_ip>dst_ip, u=udp, port#, sequence#, priority" << endl;
	// close file
	tracefile.close();

	return 0;
}