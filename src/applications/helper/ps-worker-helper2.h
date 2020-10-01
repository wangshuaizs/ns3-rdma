/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Author: Mohamed Amine Ismail <amine.ismail@sophia.inria.fr>
 */
#ifndef PS_WORKER_HELPER2_H
#define PS_WORKER_HELPER2_H

#include <stdint.h>
#include "ns3/application-container.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/ipv4-address.h"
#include "ns3/worker2.h"
#include "ns3/ps2.h"
namespace ns3 {
/**
 * \brief Create a server application which waits for input udp packets
 *        and uses the information carried into their payload to compute
 *        delay and to determine if some packets are lost.
 */
class PS2Helper
{
public:
  /**
   * Create PSHelper2 which will make life easier for people trying
   * to set up simulations with udp-client-server application.
   *
   */
	PS2Helper();

  /**
   * Create PSHelper2 which will make life easier for people trying
   * to set up simulations with udp-client-server application.
   *
   * \param port The port the server will wait on for incoming packets
   */
	PS2Helper(uint16_t port);

  /**
   * Record an attribute to be set in each Application after it is is created.
   *
   * \param name the name of the attribute to set
   * \param value the value of the attribute to set
   */
  void SetAttribute (std::string name, const AttributeValue &value);

  /**
   * Create one udp server application on each of the Nodes in the
   * NodeContainer.
   *
   * \param c The nodes on which to create the Applications.  The nodes
   *          are specified by a NodeContainer.
   * \returns The applications created, one Application per Node in the
   *          NodeContainer.
   */
  ApplicationContainer Install (NodeContainer c);
  Ptr<PS2> GetServer (void);
private:
  ObjectFactory m_factory;
  Ptr<PS2> m_server;
};

/**
 * \brief Create a client application which sends udp packets carrying
 *  a 32bit sequence number and a 64 bit time stamp.
 *
 */
class Worker2Helper
{

public:
  /**
   * Create PSHelper which will make life easier for people trying
   * to set up simulations with udp-client-server.
   *
   */
	Worker2Helper();

  /**
   *  Create PSHelper which will make life easier for people trying
   * to set up simulations with udp-client-server.
   *
   * \param ip The IP address of the remote udp server
   * \param port The port number of the remote udp server
   */

	Worker2Helper(Ipv4Address ip, uint16_t port);
	Worker2Helper(Ipv4Address ip, uint16_t port, uint16_t pg);
	Worker2Helper(Ipv4Address ip, uint16_t port, uint16_t pg, uint64_t index_order_address, uint64_t para_sizes);
	Worker2Helper(Ipv6Address ip, uint16_t port);
	Worker2Helper(Address ip, uint16_t port);

  /**
   * Record an attribute to be set in each Application after it is is created.
   *
   * \param name the name of the attribute to set
   * \param value the value of the attribute to set
   */
  void SetAttribute (std::string name, const AttributeValue &value);

  /**
     * \param c the nodes
     *
     * Create one udp client application on each of the input nodes
     *
     * \returns the applications created, one application per input node.
     */
  ApplicationContainer Install (NodeContainer c);

private:
  ObjectFactory m_factory;
};

} // namespace ns3

#endif /* PS_WORKER_HELPER2_H */
