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

#include "ns3/double.h"
#include "ns3/simulator.h"

#include "satellite-mobility-model.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SatelliteMobilityModel");

NS_OBJECT_ENSURE_REGISTERED (SatelliteMobilityModel);

TypeId
SatelliteMobilityModel::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::SatelliteMobilityModel")
    .SetParent<MobilityModel> ()
    .SetGroupName ("Leo")
    .AddConstructor<SatelliteMobilityModel> ()
    .AddAttribute ("Altitude",
                   "A height from the earth's surface in kilometers",
                   DoubleValue (1000.0),
                   MakeDoubleAccessor (&SatelliteMobilityModel::SetAltitude,
                   		       &SatelliteMobilityModel::GetAltitude),
                   MakeDoubleChecker<double> ())
    // TODO check value limits
    .AddAttribute ("Inclination",
                   "The inclination of the orbital plane in degrees",
                   DoubleValue (10.0),
                   MakeDoubleAccessor (&SatelliteMobilityModel::SetInclination,
                   		       &SatelliteMobilityModel::GetInclination),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Precision",
                   "The time precision with which to compute position updates. 0 means arbitrary precision",
                   TimeValue (Seconds (1)),
                   MakeTimeAccessor (&SatelliteMobilityModel::m_precision),
                   MakeTimeChecker ())
    ;
  return tid;
}


SatelliteMobilityModel::SatelliteMobilityModel() : MobilityModel (), m_longitude (0.0), m_offset (0.0), m_position ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

SatelliteMobilityModel::~SatelliteMobilityModel()
{
}

Vector3D
CrossProduct (const Vector3D &l, const Vector3D &r)
{
  return Vector3D (l.y * r.z - l.z * r.y,
		   l.z * r.x - l.x * r.z,
		   l.x * r.y - l.y * r.x);
}

Vector3D
Product (const double &l, const Vector3D &r)
{
  return Vector3D (l * r.x,
		   l * r.y,
		   l * r.z);
}

double
DotProduct (const Vector3D &l, const Vector3D &r)
{
  return (l.x* r.x) + (l.y*r.y) + (l.z*r.z);
}

double
SatelliteMobilityModel::GetSpeed () const
{
  return sqrt (LEO_EARTH_GM_KM_E10 / m_orbitHeight) * 1e5;
}

Vector
SatelliteMobilityModel::DoGetVelocity () const
{
  Vector3D pos = DoGetPosition ();
  pos = Vector3D (pos.x / pos.GetLength (), pos.y / pos.GetLength (), pos.z / pos.GetLength ());
  Vector3D heading = CrossProduct (PlaneNorm (), pos);
  return Product (GetSpeed (), heading);
}

Vector3D
SatelliteMobilityModel::PlaneNorm () const
{
  double lon = CalcLongitude ();
  return Vector3D (sin (m_inclination) * sin (lon),
  sin (-m_inclination) * cos (lon),
  cos (m_inclination));
}

double
SatelliteMobilityModel::GetProgress (Time t) const
{
  // TODO use nanos or ms instead? does it give higher precision?
  int sign = 1;
  // ensure correct gradient (not against earth rotation)
  if (m_inclination > M_PI/2)
    {
      sign = -1;
    }
  return sign * (((GetSpeed () * t.GetSeconds ()) / (m_orbitHeight * 1000))) + m_offset;
}


Vector3D 
SatelliteMobilityModel::RotatePlane(double a, const Vector3D &x) const 
{
  Vector3D n = PlaneNorm();
  double cos_a = cos(a);
  double sin_a = sin(a);
  double dot = DotProduct(n, x);
  
  Vector3D term1 = Product(dot, n);             
  Vector3D term2 = Product(cos_a, x - term1);   
  Vector3D term3 = Product(sin_a, CrossProduct(n, x));

  return term1 + term2 + term3;
}


double
SatelliteMobilityModel::CalcLongitude () const //Consider rotation
{
  // return m_longitude + ((Simulator::Now ().GetDouble () / Hours (24).GetDouble ()) * 2 * M_PI);
  return m_longitude;
}

Vector
SatelliteMobilityModel::CalcPosition (Time t) const
{
  double lon = CalcLongitude ();
  // account for orbit latitude and earth rotation offset
  Vector3D x = Product (m_orbitHeight*1000, Vector3D (cos (m_inclination) * cos (lon+M_PI/2),
  			       cos (m_inclination) * sin (lon+M_PI/2),
  			       sin (m_inclination)));


  return RotatePlane (GetProgress (t), x);
}

Vector SatelliteMobilityModel::Update ()
{
  m_position = CalcPosition (Simulator::Now ());
  NotifyCourseChange ();

  if (m_precision > Seconds (0))
    {
      Simulator::Schedule (m_precision, &SatelliteMobilityModel::Update, this);
    }

  return m_position;
}

Vector
SatelliteMobilityModel::DoGetPosition (void) const
{
  if (m_precision == Time (0))
    {
      // Notice: NotifyCourseChange () will not be called
      return CalcPosition (Simulator::Now ());
    }
  return m_position;
}

void
SatelliteMobilityModel::DoSetPosition (const Vector &position)
{
  // use first element of position vector as longitude, second for latitude
  // this works nicely with MobilityHelper and GetPostion will still get the
  // correct position, but be aware that it will not be the same as supplied to
  // SetPostion
  m_longitude = position.x;
  m_offset = position.y;
  Update ();
}

double SatelliteMobilityModel::GetRaan(const Vector &position)
{
  return m_raan = position.x;
}

double SatelliteMobilityModel::GetTrueAnomaly(const Vector &position)
{
  return m_trueAnomaly = position.y;
}

double SatelliteMobilityModel::GetAltitude () const
{
  return m_orbitHeight - LEO_EARTH_RAD_KM;
}

void SatelliteMobilityModel::SetAltitude (double h)
{
  m_orbitHeight = LEO_EARTH_RAD_KM + h;
  Update ();
}

double SatelliteMobilityModel::GetInclination () const
{
  return (m_inclination / M_PI) * 180.0;
}

void SatelliteMobilityModel::SetInclination (double incl)
{
  NS_ASSERT_MSG (incl != 0.0, "Plane must not be orthogonal to axis");
  m_inclination = (incl / 180.0) * M_PI;
  Update ();
}

Vector
SatelliteMobilityModel::GetPositionAtTime(Time futureTime) const
{
    NS_LOG_FUNCTION(this << futureTime);
    
    if (futureTime <= Simulator::Now())
    {
        return DoGetPosition();
    }

    double lon = CalcLongitude();
    
    Vector3D refPos = Product(m_orbitHeight * 1000, 
                              Vector3D(cos(m_inclination) * cos(lon + M_PI/2),
                                       cos(m_inclination) * sin(lon + M_PI/2),
                                       sin(m_inclination)));
    
    double futureProgress = GetProgress(futureTime);
    
    Vector3D futurePos = RotatePlane(futureProgress, refPos);
    
    return Vector(futurePos.x, futurePos.y, futurePos.z);
}

Ptr<MobilityModel>
SatelliteMobilityModel::Copy() const
{
  Ptr<SatelliteMobilityModel> copy = CreateObject<SatelliteMobilityModel>();

  copy->m_orbitHeight = m_orbitHeight;
  copy->m_inclination = m_inclination;
  copy->m_longitude = m_longitude;
  copy->m_offset = m_offset;
  copy->m_position = m_position;
  copy->m_precision = m_precision;
  copy->m_raan = m_raan;
  copy->m_trueAnomaly = m_trueAnomaly;

  return copy;
}

};
