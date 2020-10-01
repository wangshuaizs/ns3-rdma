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
#include "ps-worker-helper2.h"
#include "ns3/worker2.h"
#include "ns3/ps2.h"
#include "ns3/uinteger.h"
#include "ns3/string.h"

namespace ns3 {

PS2Helper::PS2Helper ()
{
}

PS2Helper::PS2Helper (uint16_t port)
{
  m_factory.SetTypeId (PS2::GetTypeId ());
  SetAttribute ("Port", UintegerValue (port));
}

void
PS2Helper::SetAttribute (std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
PS2Helper::Install (NodeContainer c)
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Node> node = *i;

      m_server = m_factory.Create<PS2> ();
      node->AddApplication (m_server);
      apps.Add (m_server);

    }
  return apps;
}

Ptr<PS2>
PS2Helper::GetServer (void)
{
  return m_server;
}

Worker2Helper::Worker2Helper ()
{
}

Worker2Helper::Worker2Helper (Address address, uint16_t port)
{
  m_factory.SetTypeId (Worker2::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (address));
  SetAttribute ("RemotePort", UintegerValue (port));
}

Worker2Helper::Worker2Helper (Ipv4Address address, uint16_t port)
{
  m_factory.SetTypeId (Worker2::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (Address(address)));
  SetAttribute ("RemotePort", UintegerValue (port));
}

Worker2Helper::Worker2Helper (Ipv4Address address, uint16_t port, uint16_t pg)
{
	m_factory.SetTypeId (Worker2::GetTypeId ());
	SetAttribute ("RemoteAddress", AddressValue (Address(address)));
	SetAttribute ("RemotePort", UintegerValue (port));
	SetAttribute ("PriorityGroup", UintegerValue (pg));
}

Worker2Helper::Worker2Helper (Ipv4Address address, uint16_t port, uint16_t pg, uint64_t index_order_address, uint64_t para_sizes)
{
	m_factory.SetTypeId (Worker2::GetTypeId ());
	SetAttribute ("RemoteAddress", AddressValue (Address(address)));
	SetAttribute ("RemotePort", UintegerValue (port));
	SetAttribute ("PriorityGroup", UintegerValue (pg));
  SetAttribute ("IndexOrder", UintegerValue (index_order_address));
  SetAttribute ("ParameterSizes", UintegerValue (para_sizes));
}

Worker2Helper::Worker2Helper (Ipv6Address address, uint16_t port)
{
  m_factory.SetTypeId (Worker2::GetTypeId ());
  SetAttribute ("RemoteAddress", AddressValue (Address(address)));
  SetAttribute ("RemotePort", UintegerValue (port));
}

void
Worker2Helper::SetAttribute(std::string name, const AttributeValue &value)
{
  m_factory.Set (name, value);
}

ApplicationContainer
Worker2Helper::Install (NodeContainer c)
{
  ApplicationContainer apps;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      Ptr<Node> node = *i;
      Ptr<Worker2> client = m_factory.Create<Worker2> ();
      node->AddApplication (client);
      apps.Add (client);
    }
  return apps;
}

} // namespace ns3
