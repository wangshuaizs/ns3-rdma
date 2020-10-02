/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007,2008,2009 INRIA, UDCAST
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
 *
 */

#ifndef WORKER2_H
#define WORKER2_H

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/ipv4-address.h"
#include "packet-loss-counter.h"

namespace ns3 {

class Socket;
class Packet;

/**
 * \ingroup psworker
 * \class Worker2
 * \brief A parameter server. Sends UDP packet carrying sequence number and time stamp
 *  in their payloads
 *
 */
class Worker2 : public Application
{
public:
  static TypeId
  GetTypeId (void);

  Worker2 ();

  virtual ~Worker2 ();

  /**
   * \brief set the remote address and port
   * \param ip remote IP address
   * \param port remote port
   */
  void SetRemote (Ipv4Address ip, uint16_t port);
  void SetRemote (Ipv6Address ip, uint16_t port);
  void SetRemote (Address ip, uint16_t port);
  void SetPG (uint16_t pg);

protected:
  virtual void DoDispose (void);

private:

  virtual void GetParameters (void);
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTransmit (Time dt);
  void Send (void);
  void Reset (Ptr<Socket> socket);

  uint32_t m_count;
  uint64_t m_allowed;
  Time m_interval;
  uint32_t m_size;

  uint32_t m_sent;
  Ptr<Socket> m_socket;
  Address m_peerAddress;
  uint16_t m_peerPort;
  EventId m_sendEvent;

  uint16_t m_pg;
  uint16_t m_ps_id;
  uint16_t m_worker_id;
  uint64_t m_index_order_address;
  uint32_t* m_index_order;
  uint64_t m_parameter_sizes_address;
  std::vector<uint32_t> m_parameter_sizes;
  uint64_t m_op_time_address;
  std::vector<uint32_t> m_op_times;
  uint64_t m_fp_finish_times_address;
  uint64_t* m_fp_finish_times;

  uint16_t m_num_layers;
  uint16_t m_num_servers;
  uint16_t m_num_priorities;
  uint32_t m_sent_paras;

};

} // namespace ns3

#endif /* WORKER2_H */
