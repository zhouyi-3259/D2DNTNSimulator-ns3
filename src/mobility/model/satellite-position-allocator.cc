/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Tim Schubert <ns-3-leo@timschubert.net>
 */

#include "math.h"

#include "ns3/integer.h"
#include "satellite-position-allocator.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (SatelliteAllocator);

SatelliteAllocator::SatelliteAllocator ()
  : m_lastOrbit (0), m_lastSatellite (0),
    m_useCustomParameters(false), m_customOrbitIndex(0), m_customSatelliteIndex(0)
{}

SatelliteAllocator::~SatelliteAllocator ()
{}

TypeId
SatelliteAllocator::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::SatelliteAllocator")
    .SetParent<PositionAllocator> ()
    .SetGroupName ("Leo")
    .AddConstructor<SatelliteAllocator> ()
    .AddAttribute ("NumOrbits",
                   "The number of orbits",
                   IntegerValue (1),
                   MakeIntegerAccessor (&SatelliteAllocator::m_numOrbits),
                   MakeIntegerChecker<uint16_t> ())
    .AddAttribute ("NumSatellites",
                   "The number of satellites per orbit",
                   IntegerValue (1),
                   MakeIntegerAccessor (&SatelliteAllocator::m_numSatellites),
                   MakeIntegerChecker<uint16_t> ())
    .AddAttribute ("HarmonicFactor",
                   "Harmonic factor for phase distribution",
                   IntegerValue (1),
                   MakeIntegerAccessor (&SatelliteAllocator::m_harmonicFactor),
                   MakeIntegerChecker<uint16_t> ())
  ;
  return tid;
}

void
SatelliteAllocator::SetCustomParameters(
    const std::vector<double>& customRaanList,
    const std::vector<std::vector<double>>& customTrueAnomalyList)
{
  m_customRaanList = customRaanList;
  m_customTrueAnomalyList = customTrueAnomalyList;
  m_useCustomParameters = true;
  m_customOrbitIndex = 0;
  m_customSatelliteIndex = 0;
}


int64_t
SatelliteAllocator::AssignStreams (int64_t stream)
{
  return -1;
}

Vector
SatelliteAllocator::GetNext () const
{
  if (m_useCustomParameters)
  {
    if (m_customRaanList.empty() || m_customTrueAnomalyList.empty()) 
    {
      NS_FATAL_ERROR("Custom parameters are enabled but lists are empty");
    }
    
    uint64_t orbitIdx = m_customOrbitIndex % m_customRaanList.size();

    const std::vector<double>& satList = m_customTrueAnomalyList[orbitIdx];
    
    uint64_t satIdx = m_customSatelliteIndex % satList.size();
    
    double raan = m_customRaanList[orbitIdx];
    double trueAnomaly = satList[satIdx];
    
    Vector next = Vector(raan, trueAnomaly, 0.0);
    
    m_customSatelliteIndex++;
    if (m_customSatelliteIndex >= satList.size()) 
    {
      m_customSatelliteIndex = 0;
      m_customOrbitIndex = (m_customOrbitIndex + 1) % m_customRaanList.size();
    }
    
    return next;
  }


  double raan = 2 * M_PI * (m_lastOrbit / (double) m_numOrbits);
  // double trueAnomaly = 2 * M_PI * (m_lastSatellite / (double) m_numSatellites);
  double trueAnomaly = 2 * M_PI * m_harmonicFactor * (m_lastSatellite / (double) m_numSatellites);
  Vector next = Vector(raan, trueAnomaly, 0.0);



  if (m_lastSatellite + 1 == m_numSatellites)
    {
      m_lastOrbit = (m_lastOrbit + 1) % m_numOrbits;
    }
  m_lastSatellite = (m_lastSatellite + 1) % m_numSatellites;

  return next;
}

};
