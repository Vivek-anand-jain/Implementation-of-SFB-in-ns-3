/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 NITK Surathkal
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
 * Authors: Vivek Jain <jain.vivek.anand@gmail.com>
 *          Mohit P. Tahiliani <tahiliani@nitk.edu.in>
 */

#include "ns3/log.h"
#include "ns3/enum.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/simulator.h"
#include "ns3/abort.h"
#include "sfb-queue-disc.h"
#include "ns3/ipv4-packet-filter.h"
#include "ns3/drop-tail-queue.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SFBQueueDisc");

NS_OBJECT_ENSURE_REGISTERED (SFBQueueDisc);

TypeId SFBQueueDisc::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SFBQueueDisc")
    .SetParent<QueueDisc> ()
    .SetGroupName ("TrafficControl")
    .AddConstructor<SFBQueueDisc> ()
    .AddAttribute ("Mode",
                   "Determines unit for QueueLimit",
                   EnumValue (Queue::QUEUE_MODE_PACKETS),
                   MakeEnumAccessor (&SFBQueueDisc::SetMode),
                   MakeEnumChecker (Queue::QUEUE_MODE_BYTES, "QUEUE_MODE_BYTES",
                                    Queue::QUEUE_MODE_PACKETS, "QUEUE_MODE_PACKETS"))
    .AddAttribute ("QueueLimit",
                   "Queue limit in bytes/packets",
                   UintegerValue (25),
                   MakeUintegerAccessor (&SFBQueueDisc::SetQueueLimit),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("MeanPktSize",
                   "Average of packet size",
                   UintegerValue (1000),
                   MakeUintegerAccessor (&SFBQueueDisc::m_meanPktSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("Increment",
                   "Pmark increment value",
                   DoubleValue (0.0025),
                   MakeDoubleAccessor (&SFBQueueDisc::m_increment),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Decrement",
                   "Pmark decrement Value",
                   DoubleValue (0.00025),
                   MakeDoubleAccessor (&SFBQueueDisc::m_decrement),
                   MakeDoubleChecker<double> ())
  ;

  return tid;
}

SFBQueueDisc::SFBQueueDisc () :
  QueueDisc ()
{
  NS_LOG_FUNCTION (this);
  m_uv = CreateObject<UniformRandomVariable> ();
}

SFBQueueDisc::~SFBQueueDisc ()
{
  NS_LOG_FUNCTION (this);
}

void
SFBQueueDisc::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_uv = 0;
  QueueDisc::DoDispose ();
}

void
SFBQueueDisc::SetMode (Queue::QueueMode mode)
{
  NS_LOG_FUNCTION (this << mode);
  m_mode = mode;
}

Queue::QueueMode
SFBQueueDisc::GetMode (void)
{
  NS_LOG_FUNCTION (this);
  return m_mode;
}

void
SFBQueueDisc::SetQueueLimit (uint32_t lim)
{
  NS_LOG_FUNCTION (this << lim);
  m_queueLimit = lim;
}

uint32_t
SFBQueueDisc::GetQueueSize (void)
{
  NS_LOG_FUNCTION (this);
  if (GetMode () == Queue::QUEUE_MODE_BYTES)
    {
      return GetInternalQueue (0)->GetNBytes ();
    }
  else if (GetMode () == Queue::QUEUE_MODE_PACKETS)
    {
      return GetInternalQueue (0)->GetNPackets ();
    }
  else
    {
      NS_ABORT_MSG ("Unknown Blue mode.");
    }
}

SFBQueueDisc::Stats
SFBQueueDisc::GetStats ()
{
  NS_LOG_FUNCTION (this);
  return m_stats;
}

int64_t
SFBQueueDisc::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uv->SetStream (stream);
  return 1;
}

void
SFBQueueDisc::InitializeParams (void)
{
  InitializeBins();
  m_stats.forcedDrop = 0;
  m_stats.unforcedDrop = 0;
  m_binSize = 1.0 / SFB_BINS * GetQueueSize(); 
}

void
SFBQueueDisc::InitializeBins (void)
{
  for (uint32_t i = 0; i < SFB_LEVELS; i++)
    {
      for (uint32_t j = 0; j < SFB_BINS; j++)
        {
          m_bins[i][j].packets = 0;
          m_bins[i][j].pmark = 0;
        }
    }
}

bool
SFBQueueDisc::DoEnqueue (Ptr<QueueDiscItem> item)
{
  NS_LOG_FUNCTION (this << item);

  uint32_t nQueued = GetQueueSize ();

  if ((GetMode () == Queue::QUEUE_MODE_PACKETS && nQueued >= m_queueLimit)
      || (GetMode () == Queue::QUEUE_MODE_BYTES && nQueued + item->GetPacketSize () > m_queueLimit))
    {
      IncrementBinsPmarks(item);
      m_stats.forcedDrop++;
      Drop (item);
      return false;
    }

  for (uint32_t i = 0; i < SFB_LEVELS; i++)
    {
      uint32_t hashed = 1;//GetHash(i, item);
      if (m_bins[i][hashed].packets == 0)
        {
          DecrementBinPmark(i, hashed);
        }
      else if (m_bins[i][hashed].packets > m_binSize)
        {
          IncrementBinPmark(i, hashed);
        }
    }

  if (GetMinProbability (item) == 1.0)
    {
      //rateLimit();
    }
  else if (DropEarly (item))
    {
      Drop (item);
      return false;
    }

  bool isEnqueued = GetInternalQueue (0)->Enqueue (item);
  if (isEnqueued == true)
    {
      IncrementBinsQueueLength(item);
    }
  return isEnqueued;
}

double
SFBQueueDisc::GetMinProbability (Ptr<QueueDiscItem> item)
{
  NS_LOG_FUNCTION (this);
  double u =  1.0;
  for (uint32_t i = 0; i < SFB_LEVELS; i++)
    {
      uint32_t hashed = 1;//GetHash(i, item);
      if (u > m_bins[i][hashed].pmark)
        {
          u = m_bins[i][hashed].pmark;
        }
    }
  return u;
}

bool
SFBQueueDisc::DropEarly (Ptr<QueueDiscItem> item)
{
  NS_LOG_FUNCTION (this);
  double u =  m_uv->GetValue ();
  if (u <= GetMinProbability(item))
    {
      return true;
    }
  return false;
}

void
SFBQueueDisc::IncrementBinsQueueLength (Ptr<QueueDiscItem> item)
{
  for (uint32_t i = 0; i < SFB_LEVELS; i++)
    {
      uint32_t hashed = 1;//GetHash(i, item);
      m_bins[i][hashed].packets++;
      if (m_bins[i][hashed].packets > m_binSize)
        {
          IncrementBinPmark(i, hashed);
        }
    }
}

void
SFBQueueDisc::IncrementBinsPmarks (Ptr<QueueDiscItem> item)
{
  for (uint32_t i = 0; i < SFB_LEVELS; i++)
    {
      uint32_t hashed = 1;//GetHash(i, item);
      IncrementBinPmark(i, hashed);
    }
}

void
SFBQueueDisc::IncrementBinPmark (uint32_t i, uint32_t j)
{
  m_bins[i][j].pmark += m_increment;
  if (m_bins[i][j].pmark > 1.0)
    {
      m_bins[i][j].pmark = 1.0;
    }
}

void
SFBQueueDisc::DecrementBinsQueueLength (Ptr<QueueDiscItem> item)
{
  for (uint32_t i = 0; i < SFB_LEVELS; i++)
    {
      uint32_t hashed = 1;//GetHash(i, item);
      m_bins[i][hashed].packets--;
      if (m_bins[i][hashed].packets < 0)
        {
          m_bins[i][hashed].packets = 0;
        }
    }
}

void
SFBQueueDisc::DecrementBinPmark (uint32_t i, uint32_t j)
{
  m_bins[i][j].pmark -= m_decrement;
  if (m_bins[i][j].pmark < 0.0)
    {
      m_bins[i][j].pmark = 0.0;
    }
}

Ptr<QueueDiscItem>
SFBQueueDisc::DoDequeue (void)
{
  NS_LOG_FUNCTION (this);

  Ptr<QueueDiscItem> item = StaticCast<QueueDiscItem> (GetInternalQueue (0)->Dequeue ());

  NS_LOG_LOGIC ("Popped " << item);

  if (item)
    {
      DecrementBinsQueueLength(item);
    }

  return item;
}

Ptr<const QueueDiscItem>
SFBQueueDisc::DoPeek () const
{
  NS_LOG_FUNCTION (this);
  if (GetInternalQueue (0)->IsEmpty ())
    {
      NS_LOG_LOGIC ("Queue empty");
      return 0;
    }

  Ptr<const QueueDiscItem> item = StaticCast<const QueueDiscItem> (GetInternalQueue (0)->Peek ());

  NS_LOG_LOGIC ("Number packets " << GetInternalQueue (0)->GetNPackets ());
  NS_LOG_LOGIC ("Number bytes " << GetInternalQueue (0)->GetNBytes ());

  return item;
}

bool
SFBQueueDisc::CheckConfig (void)
{
  NS_LOG_FUNCTION (this);
  if (GetNQueueDiscClasses () > 0)
    {
      NS_LOG_ERROR ("SFBQueueDisc cannot have classes");
      return false;
    }

  if (GetNPacketFilters () == 0)
    {
      Ptr<SFBIpv4PacketFilter> ipv4Filter = CreateObject<SFBIpv4PacketFilter> ();
      AddPacketFilter (ipv4Filter);
    }

  if (GetNPacketFilters () != 1)
    {
      NS_LOG_ERROR ("SFBQueueDisc needs 1 filter");
      return false;
    }

  if (GetNInternalQueues () == 0)
    {
      // create a DropTail queue
      Ptr<Queue> queue = CreateObjectWithAttributes<DropTailQueue> ("Mode", EnumValue (m_mode));
      if (m_mode == Queue::QUEUE_MODE_PACKETS)
        {
          queue->SetMaxPackets (m_queueLimit);
        }
      else
        {
          queue->SetMaxBytes (m_queueLimit);
        }
      AddInternalQueue (queue);
    }

  if (GetNInternalQueues () != 1)
    {
      NS_LOG_ERROR ("SFBQueueDisc needs 1 internal queue");
      return false;
    }

  if (GetInternalQueue (0)->GetMode () != m_mode)
    {
      NS_LOG_ERROR ("The mode of the provided queue does not match the mode set on the SFBQueueDisc");
      return false;
    }

  if ((m_mode ==  Queue::QUEUE_MODE_PACKETS && GetInternalQueue (0)->GetMaxPackets () < m_queueLimit)
      || (m_mode ==  Queue::QUEUE_MODE_BYTES && GetInternalQueue (0)->GetMaxBytes () < m_queueLimit))
    {
      NS_LOG_ERROR ("The size of the internal queue is less than the queue disc limit");
      return false;
    }

  return true;
}

} //namespace ns3
