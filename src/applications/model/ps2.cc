/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 *  Copyright (c) 2007,2008,2009 INRIA, UDCAST
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Amine Ismail <amine.ismail@sophia.inria.fr>
 *                      <amine.ismail@udcast.com>
 */

#include "ns3/log.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/inet6-socket-address.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/random-variable.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "packet-loss-counter.h"

#include "ns3/seq-ts-header.h"
#include "ps2.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PS2");
NS_OBJECT_ENSURE_REGISTERED (PS2);


TypeId
PS2::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PS2")
    .SetParent<Application> ()
    .AddConstructor<PS2> ()
    .AddAttribute ("Port",
                   "Port on which we listen for incoming packets.",
                   UintegerValue (100),
                   MakeUintegerAccessor (&PS2::m_port),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("PacketWindowSize",
                   "The size of the window used to compute the packet loss. This value should be a multiple of 8.",
                   UintegerValue (32),
                   MakeUintegerAccessor (&PS2::GetPacketWindowSize,
                                         &PS2::SetPacketWindowSize),
                   MakeUintegerChecker<uint16_t> (8,256))
    .AddAttribute ("PSID",
                   "PSID.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&PS2::m_ps_id),
                   MakeUintegerChecker<uint16_t> (0,65535))
    .AddAttribute ("PacketSize",
                   "Size of packets received.",
                   UintegerValue (1024),
                   MakeUintegerAccessor (&PS2::m_size),
                   MakeUintegerChecker<uint32_t> (14,1500))
    .AddAttribute ("ParameterSizes",
                   "ParameterSizes.",
                   UintegerValue (0),
                   MakeUintegerAccessor (&PS2::m_parameter_sizes_address),
                   MakeUintegerChecker<uint64_t> (0, -1))
    .AddAttribute ("NumLayers",
                   "NumLayers",
                   UintegerValue (0),
                   MakeUintegerAccessor (&PS2::m_num_layers),
                   MakeUintegerChecker<uint16_t> (0,65535))
    .AddAttribute ("NumServers",
                   "NumServers",
                   UintegerValue (0),
                   MakeUintegerAccessor (&PS2::m_num_servers),
                   MakeUintegerChecker<uint16_t> (0,65535))
  ;
  return tid;
}

PS2::PS2 ()
  : m_lossCounter (0)
{
  NS_LOG_FUNCTION (this);
  m_received=0;
  m_num_ready_paras = 0;
}

PS2::~PS2 ()
{
  NS_LOG_FUNCTION (this);
}

uint16_t
PS2::GetPacketWindowSize () const
{
  return m_lossCounter.GetBitMapSize ();
}

void
PS2::SetPacketWindowSize (uint16_t size)
{
  m_lossCounter.SetBitMapSize (size);
}

uint32_t
PS2::GetLost (void) const
{
  return m_lossCounter.GetLost ();
}

uint32_t
PS2::GetReceived (void) const
{

  return m_received;

}

void
PS2::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
PS2::GetParameters (void)
{
  uint32_t* tmp_addr = (uint32_t*) m_parameter_sizes_address;
  for (int k = 0; k < m_num_layers; k++) {
    m_parameter_sizes.push_back(tmp_addr[k]);
    m_partition_ready_bars.push_back(ceil(tmp_addr[k]*1.0/m_size)*(m_num_servers-1));
  }
  m_num_patitions.resize(m_num_layers);
  m_para_ready_times.resize(m_num_layers);
  /*for (int k = 0; k < m_num_layers; k++)
    std::cout << m_parameter_sizes[k] << " ";
  std::cout << "\n";

  for (auto& it : m_partition_ready_bars)
    std::cout << it << " ";
  std::cout << "\n";*/
}

void
PS2::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  GetParameters();

  if (m_socket == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);
      InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (),
                                                   m_port);
      m_socket->Bind (local);
    }

  m_socket->SetRecvCallback (MakeCallback (&PS2::HandleRead, this));

  if (m_socket6 == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
      m_socket6 = Socket::CreateSocket (GetNode (), tid);
      Inet6SocketAddress local = Inet6SocketAddress (Ipv6Address::GetAny (),
                                                   m_port);
      m_socket6->Bind (local);
    }

  m_socket6->SetRecvCallback (MakeCallback (&PS2::HandleRead, this));

}

void
PS2::StopApplication ()
{
  NS_LOG_FUNCTION (this);

  if (m_socket != 0)
    {
      m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
}

void
PS2::HandleRead (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      if (packet->GetSize () > 0)
        {
          SeqTsHeader seqTs;
          packet->RemoveHeader (seqTs);
          uint32_t currentSequenceNumber = seqTs.GetSeq ();
          if (InetSocketAddress::IsMatchingType (from))
            {
              NS_LOG_INFO ("TraceDelay: RX " << packet->GetSize () <<
                           " bytes from "<< InetSocketAddress::ConvertFrom (from).GetIpv4 () <<
                           " Sequence Number: " << currentSequenceNumber <<
                           " Uid: " << packet->GetUid () <<
                           " TXtime: " << seqTs.GetTs () <<
                           " RXtime: " << Simulator::Now () <<
                           " Delay: " << Simulator::Now () - seqTs.GetTs ());
              uint32_t para_id = seqTs.GetParaID();
              m_num_patitions[para_id]++;
              if (m_num_patitions[para_id] == m_partition_ready_bars[para_id]) {
                m_para_ready_times[para_id] = Simulator::Now().GetMicroSeconds();
                m_num_ready_paras++;
                std::cout << "At " << Simulator::Now().GetMicroSeconds() << " us " << para_id << " @ " << m_ps_id << " has finished receving.\n";

                if (m_num_ready_paras == m_num_layers) { //All parameters have been recieved by now
                    std::cout << "PS " << m_ps_id << " finished BP at " << Simulator::Now().GetMicroSeconds() << " us.\n";
                }
              }
            }
          else if (Inet6SocketAddress::IsMatchingType (from))
            {
              NS_LOG_INFO ("TraceDelay: RX " << packet->GetSize () <<
                           " bytes from "<< Inet6SocketAddress::ConvertFrom (from).GetIpv6 () <<
                           " Sequence Number: " << currentSequenceNumber <<
                           " Uid: " << packet->GetUid () <<
                           " TXtime: " << seqTs.GetTs () <<
                           " RXtime: " << Simulator::Now () <<
                           " Delay: " << Simulator::Now () - seqTs.GetTs ());
            }

          m_lossCounter.NotifyReceived (currentSequenceNumber);
          m_received++;
        }
    }
}

void
PS2::SetRemote (Ipv4Address ip, uint16_t port)
{
	m_peerAddress = Address(ip);
	m_peerPort = port;
}


} // Namespace ns3
