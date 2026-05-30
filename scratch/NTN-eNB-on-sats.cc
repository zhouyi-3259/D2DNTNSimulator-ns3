/*
 * Copyright (c) 2011-2018 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Authors: 
 */

#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/mobility-model.h"
#include "ns3/point-to-point-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/spectrum-channel.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/single-model-spectrum-channel.h"
#include <ns3/lte-spectrum-value-helper.h>
#include "ns3/double.h"
#include "ns3/log.h"

#include "ns3/satellite-mobility-model.h"
#include "ns3/satellite-position-allocator.h"
#include "ns3/three-gpp-propagation-loss-model.h"

#include <chrono>
#include <iomanip>
#include <thread>
#include <cmath>



using namespace ns3;

#define EarthRad 6371000.0
//-----------------------------------------------------------------------------------------------------------------------------------------------------
//------------------------------ENB\UE Establish or Handover Trace Function----------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------------
/**
 * UE Connection established notification.
 *
 * \param context The context.
 * \param imsi The IMSI of the connected terminal.
 * \param cellid The Cell ID.
 * \param rnti The RNTI.
 */
void
NotifyConnectionEstablishedUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().As(Time::S) << " " << context << " UE IMSI " << imsi
              << ": connected to CellId " << cellid << " with RNTI " << rnti << std::endl;
}

/**
 * UE Start Handover notification.
 *
 * \param context The context.
 * \param imsi The IMSI of the connected terminal.
 * \param cellid The actual Cell ID.
 * \param rnti The RNTI.
 * \param targetCellId The target Cell ID.
 */
void
NotifyHandoverStartUe(std::string context,
                      uint64_t imsi,
                      uint16_t cellid,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    std::cout << Simulator::Now().As(Time::S) << " " << context << " UE IMSI " << imsi
              << ": previously connected to CellId " << cellid << " with RNTI " << rnti
              << ", doing handover to CellId " << targetCellId << std::endl;
}

/**
 * UE Handover end successful notification.
 *
 * \param context The context.
 * \param imsi The IMSI of the connected terminal.
 * \param cellid The Cell ID.
 * \param rnti The RNTI.
 */
void
NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().As(Time::S) << " " << context << " UE IMSI " << imsi
              << ": successful handover to CellId " << cellid << " with RNTI " << rnti << std::endl;
}

/**
 * eNB Connection established notification.
 *
 * \param context The context.
 * \param imsi The IMSI of the connected terminal.
 * \param cellid The Cell ID.
 * \param rnti The RNTI.
 */
void
NotifyConnectionEstablishedEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().As(Time::S) << " " << context << " eNB CellId " << cellid
              << ": successful connection of UE with IMSI " << imsi << " RNTI " << rnti
              << std::endl;
}

/**
 * eNB Start Handover notification.
 *
 * \param context The context.
 * \param imsi The IMSI of the connected terminal.
 * \param cellid The actual Cell ID.
 * \param rnti The RNTI.
 * \param targetCellId The target Cell ID.
 */
void
NotifyHandoverStartEnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    std::cout << Simulator::Now().As(Time::S) << " " << context << " eNB CellId " << cellid
              << ": start handover of UE with IMSI " << imsi << " RNTI " << rnti << " to CellId "
              << targetCellId << std::endl;
}

/**
 * eNB Handover end successful notification.
 *
 * \param context The context.
 * \param imsi The IMSI of the connected terminal.
 * \param cellid The Cell ID.
 * \param rnti The RNTI.
 */
void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().As(Time::S) << " " << context << " eNB CellId " << cellid
              << ": completed handover of UE with IMSI " << imsi << " RNTI " << rnti << std::endl;
}

/**
 * Handover failure notification
 *
 * \param context The context.
 * \param imsi The IMSI of the connected terminal.
 * \param cellid The Cell ID.
 * \param rnti The RNTI.
 */
void
NotifyHandoverFailure(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::cout << Simulator::Now().As(Time::S) << " " << context << " eNB CellId " << cellid
              << " IMSI " << imsi << " RNTI " << rnti << " handover failure" << std::endl;
}


Time udpInterval;
Time lastStatisticsTime;
uint64_t totalRxBytesInWindow = 0;
uint64_t totalRxPacketsInWindow = 0;
uint64_t totalTxPacketsExpectedInWindow = 0;
Time statisticsWindow = Seconds(1.0);

Ptr<UdpClient> udpClientApp;


void PeriodicStatistics() {
    Time currentTime = Simulator::Now();
    double timeWindow = (currentTime - lastStatisticsTime).GetSeconds();
    double throughput = (totalRxBytesInWindow * 8.0) / timeWindow / 1.0e6; // Mbps
    double lossRate = 0.0;
    double expectedPackets = timeWindow / udpInterval.GetSeconds(); 

    if (expectedPackets > 0) {
        lossRate = ((expectedPackets - totalRxPacketsInWindow) / expectedPackets) * 100.0;
        lossRate = std::max(0.0, std::min(100.0, lossRate));
    }

    std::cout << "[" << currentTime.GetSeconds() << "s Window] "
              << "Throughput: " << throughput << " Mbps, "
              << "Loss Rate: " << lossRate << "%, "
              << "(Rx " << totalRxPacketsInWindow << " pkts, Expected ~" << expectedPackets << " pkts)"
              << std::endl;

    totalRxBytesInWindow = 0;
    totalRxPacketsInWindow = 0;

    lastStatisticsTime = currentTime;
    Simulator::Schedule(statisticsWindow, &PeriodicStatistics);
}

static void SinkRxWithAddress(std::string path, Ptr<const Packet> p, const Address &peerAddress, const Address &localAddress) {
    totalRxBytesInWindow += p->GetSize();
    totalRxPacketsInWindow++;
}


void PrintAllEnbPositions(NodeContainer enbNodes) 
{
    for (uint32_t enbIndex = 0; enbIndex < enbNodes.GetN(); ++enbIndex) 
    {
        Ptr<Node> enbNode = enbNodes.Get(enbIndex);
        Ptr<MobilityModel> mobility = enbNode->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        std::cout << "Time: " << Simulator::Now().GetSeconds() << "s | "
                  << "Satellite " << enbIndex
                  << " Position (x,y,z): (" << pos.x << ", " << pos.y << ", " << pos.z << ")"
                  << std::endl;
    }
    // Simulator::Schedule(Seconds(1.0), &PrintAllEnbPositions, enbNodes); 
}

void PrintAllUePositions(NodeContainer ueNodes) 
{
    for (uint32_t ueIndex = 0; ueIndex < ueNodes.GetN(); ++ueIndex) 
    {
        Ptr<Node> ueNode = ueNodes.Get(ueIndex);
        Ptr<MobilityModel> mobility = ueNode->GetObject<MobilityModel>();
        Vector pos = mobility->GetPosition();
        std::cout << "UE " << ueIndex
                  << " Position (x,y,z): (" << pos.x << ", " << pos.y << ", " << pos.z << ")"
                  << std::endl;
    }
}


Ptr<Node> FindEnbByCellId(NodeContainer& enbNodes, uint16_t cellId)
{
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i) 
    {
        Ptr<LteEnbNetDevice> enbDev = enbNodes.Get(i)->GetDevice(0)->GetObject<LteEnbNetDevice>();
        if (enbDev->GetCellId() == cellId) 
        {
            return enbNodes.Get(i);
        }
    }
    return nullptr;
}


std::pair<bool, double> CalculateCutoffDistance(Vector satPosition, double minElevationAngleDeg)
{
    double minElevationAngleRad = minElevationAngleDeg * M_PI / 180.0;
    double hs = satPosition.GetLength();
    
    double a = 1 + tan(minElevationAngleRad) * tan(minElevationAngleRad);
    double b = 2.0 * EarthRad * tan(minElevationAngleRad);
    double c = EarthRad * EarthRad - hs * hs;
    
    double discriminant = b * b - 4 * a * c;
    
    if (discriminant < 0) {
        return std::make_pair(false, 0.0);
    }
    
    double t1 = (-b - sqrt(discriminant)) / (2.0 * a);
    double t2 = (-b + sqrt(discriminant)) / (2.0 * a);
    
    double d1 = sqrt(t1 * t1 + (t1 * tan(minElevationAngleRad)) * (t1 * tan(minElevationAngleRad)));
    double d2 = sqrt(t2 * t2 + (t2 * tan(minElevationAngleRad)) * (t2 * tan(minElevationAngleRad)));
    
    double cutoffDistance = std::min(d1, d2);
    return std::make_pair(true, cutoffDistance);
}

void GetCutoffDistance(NodeContainer enbNodes, NodeContainer ueNodes, double minElevationAngle, double normalTxPower)
{
    for (uint32_t enbIndex = 0; enbIndex < enbNodes.GetN(); ++enbIndex) 
    {
        Ptr<Node> enbNode = enbNodes.Get(enbIndex);
        
        Ptr<MobilityModel> enbMobility = enbNode->GetObject<MobilityModel>();
        if (!enbMobility) {
            continue;
        }
        Vector enbPos = enbMobility->GetPosition();
        
        auto result = CalculateCutoffDistance(enbPos, minElevationAngle);
        bool isVisible = result.first;
        double cutoffDistance = result.second;
        
        Ptr<LteEnbNetDevice> enbDev = enbNode->GetDevice(0)->GetObject<LteEnbNetDevice>();
        if (!enbDev) {
            continue;
        }
        
        if (!isVisible) {
            if (enbDev && enbDev->GetPhy()->GetTxPower() > 0.0) {
                enbDev->GetPhy()->SetTxPower(0.0);
                std::cout << "[" << Simulator::Now().GetSeconds() << "s] ENB " 
                          << enbIndex << " closed" << std::endl;
            }
            continue;
        }
        
        bool isInRange = false;
        
        for (uint32_t ueIndex = 0; ueIndex < ueNodes.GetN(); ++ueIndex) 
        {
            Ptr<Node> ueNode = ueNodes.Get(ueIndex);
            Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
            
            if (ueMobility) {
                Vector uePos = ueMobility->GetPosition();
                double distance = CalculateDistance(enbPos, uePos);
                
                if (distance <= cutoffDistance) {
                    isInRange = true;
                    break;
                }
            }
        }
        
        if (isInRange) {
            if (enbDev->GetPhy()->GetTxPower() == 0.0) {
                enbDev->GetPhy()->SetTxPower(normalTxPower);
            }
        } else {
            if (enbDev->GetPhy()->GetTxPower() > 0.0) {
                enbDev->GetPhy()->SetTxPower(0.0);
            }
        }
    }
    
    Simulator::Schedule(Seconds(1), &GetCutoffDistance, enbNodes, ueNodes, minElevationAngle, normalTxPower);
}

void PrintCutoffDistance(NodeContainer enbNodes, double minElevationAngle)
{
    if (enbNodes.GetN() == 0) {
        std::cout << "no ENB" << std::endl;
        return;
    }
    
    Ptr<Node> enbNode = enbNodes.Get(0);
    Ptr<MobilityModel> enbMobility = enbNode->GetObject<MobilityModel>();
    
    if (!enbMobility) {
        std::cout << "ENB 0 no mobility model" << std::endl;
        return;
    }
    
    Vector enbPos = enbMobility->GetPosition();
    auto result = CalculateCutoffDistance(enbPos, minElevationAngle);
    
    if (result.first) {
        std::cout << "[" << Simulator::Now().GetSeconds() << "s] "
                  << "Angle=" << minElevationAngle << ", " 
                  << "CutoffDistance=" << result.second << "m"
                  << std::endl;
    } else {
        std::cout << "[" << Simulator::Now().GetSeconds() << "s] "
                  << "Angle=" << minElevationAngle << ", "
                  << "no visible satellite"
                  << std::endl;
    }
}


void CheckFlowStats(Ptr<FlowMonitor> monitor, 
                Ptr<Ipv4FlowClassifier> classifier, 
                double interval,
                bool& first,
                std::map<FlowId, uint64_t>& lastRxPkts,
                std::map<FlowId, uint64_t>& lastTxPkts,
                std::map<FlowId, uint64_t>& lastLostPkts)
{
    Time now = Simulator::Now();
    std::cout << "Time: " << now.GetSeconds() << "s" << std::endl;
    
    monitor->CheckForLostPackets();
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    
    for (auto iter = stats.begin(); iter != stats.end(); ++iter)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);

        uint64_t txInInterval = 0;
        uint64_t rxInInterval = 0;
        uint64_t lostInInterval = 0;
        double lossRateInInterval = 0.0;

        
        if (!first)
        {
            txInInterval = iter->second.txPackets - lastTxPkts[iter->first];
            rxInInterval = iter->second.rxPackets - lastRxPkts[iter->first];
            lostInInterval = iter->second.lostPackets - lastLostPkts[iter->first];

            double throughput = (rxInInterval * 1024 * 8.0) / (interval * 1000000.0); // Mbps
            std::cout << "  Throughput (last " << interval << "s): " << throughput << " Mbps" << std::endl;
            
            if (txInInterval > 0) 
            {
                lossRateInInterval = static_cast<double>(lostInInterval) / txInInterval * 100.0;
            }

        }

        std::cout << "Flow " << iter->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")" << std::endl;
        std::cout << "  Tx Packets: " << txInInterval << std::endl;
        std::cout << "  Rx Packets: " << rxInInterval << std::endl;
        std::cout << "  Lost Packets: " << lostInInterval << std::endl;
        std::cout << "  Loss Rate: " << lossRateInInterval << "%" << std::endl;
        std::cout << "  Mean Delay: " << iter->second.delaySum.GetSeconds() / iter->second.rxPackets << "s" << std::endl;
        std::cout << "  Mean Jitter: " << iter->second.jitterSum.GetSeconds() / (iter->second.rxPackets - 1) << "s" << std::endl;
        
        lastRxPkts[iter->first] = iter->second.rxPackets;
        lastTxPkts[iter->first] = iter->second.txPackets;
        lastLostPkts[iter->first] = iter->second.lostPackets;
    }
    std::cout << "=====================================" << std::endl;
    
    first = false;
    
    Simulator::Schedule(Seconds(interval), &CheckFlowStats, monitor, classifier, interval, std::ref(first), std::ref(lastRxPkts), std::ref(lastTxPkts), std::ref(lastLostPkts));
}



//-----------------------------------------------------------------------------------------------------
//--------------------------------------------CSHO-----------------------------------------------------
//-----------------------------------------------------------------------------------------------------


struct UeHandoverStatus2 
{
    Ptr<Node> servingEnb;
    std::unordered_map<Ptr<Node>, uint32_t> EnbIndexMap;
};

std::map<uint32_t, UeHandoverStatus2> ueHandoverStates2;

void CheckLinkCondition_closest(
    NodeContainer enbNodes, 
    NodeContainer ueNodes, 
    double minElevationAngle, 
    double earthRadius,
    double txPower,
    Ptr<LteHelper> lteHelper) 
{
    for (uint32_t ueIndex = 0; ueIndex < ueNodes.GetN(); ++ueIndex) 
    {
        Ptr<Node> ueNode = ueNodes.Get(ueIndex);
        Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
        auto& ueState = ueHandoverStates2[ueIndex];

        Ptr<Node> nearestEnb = nullptr;
        double minDistance = std::numeric_limits<double>::max();
        std::unordered_map<Ptr<Node>, uint32_t> EnbNodeToIndexMap;

        for (uint32_t enbIndex = 0; enbIndex < enbNodes.GetN(); ++enbIndex) 
        {
            Ptr<Node> enbNode = enbNodes.Get(enbIndex);
            EnbNodeToIndexMap[enbNode] = enbIndex;
            Ptr<MobilityModel> enbMobility = enbNode->GetObject<MobilityModel>();
            double distance = enbMobility->GetDistanceFrom(ueMobility);

            if (distance < minDistance) {
                minDistance = distance;
                nearestEnb = enbNode;
            }
        }

        ueState.EnbIndexMap = EnbNodeToIndexMap;

        if (ueState.servingEnb == nullptr)
        {
            ueState.servingEnb = nearestEnb;
            std::cout << "[" << Simulator::Now().GetSeconds() << "s] UE " << ueIndex 
                      << " Initial connection to Satellite" 
                      << ueState.EnbIndexMap[nearestEnb] << std::endl;
        }
        else if (ueState.servingEnb != nearestEnb)
        {
            Ptr<LteUeNetDevice> ueLteDev = ueNode->GetDevice(0)->GetObject<LteUeNetDevice>();
            Ptr<LteEnbNetDevice> sourceEnbDev = ueState.servingEnb->GetDevice(0)->GetObject<LteEnbNetDevice>();
            Ptr<LteEnbNetDevice> targetEnbDev = nearestEnb->GetDevice(0)->GetObject<LteEnbNetDevice>();

            lteHelper->HandoverRequest(MilliSeconds(10), ueLteDev, sourceEnbDev, targetEnbDev);

            uint32_t oldIndex = ueState.EnbIndexMap[ueState.servingEnb];
            uint32_t newIndex = ueState.EnbIndexMap[nearestEnb];
            std::cout << "[" << Simulator::Now().GetSeconds() << "s] UE " << ueIndex 
                      << " HO completed: Satellite" << oldIndex 
                      << " -> Satellite" << newIndex << std::endl;

            ueState.servingEnb = nearestEnb;
        }
    }

    Simulator::Schedule(MilliSeconds(100), &CheckLinkCondition_closest, enbNodes, ueNodes, 
                       minElevationAngle, earthRadius, txPower, lteHelper);
}

//-----------------------------------------------------------------------------------------------------
//--------------------------------------------MAX-visibility-HO--------------------------------
//-----------------------------------------------------------------------------------------------------

double CalculateElevationAngle(const Vector& uePos, const Vector& satPos) {
    Vector relativePos = satPos - uePos;
    double distance = relativePos.GetLength();
    
    double dotProduct = relativePos.x * uePos.x + 
                       relativePos.y * uePos.y + 
                       relativePos.z * uePos.z;
    
    double ueDistance = uePos.GetLength();
    
    double sinElevation = dotProduct / (distance * ueDistance);
    
    if (sinElevation > 1.0) sinElevation = 1.0;
    if (sinElevation < -1.0) sinElevation = -1.0;

    return std::asin(sinElevation) * 180.0 / M_PI;
}

Time CalculateRemainingVisibleTime(Ptr<Node> ueNode, Ptr<Node> enbNode, double minElevationAngle) {
    Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
    Ptr<SatelliteMobilityModel> satMobility = enbNode->GetObject<SatelliteMobilityModel>();
    
    if (!ueMobility || !satMobility) {
        return Seconds(0);
    }

    Vector uePos = ueMobility->GetPosition();
    Vector satPos = satMobility->GetPosition();

    double currentElevation = CalculateElevationAngle(uePos, satPos);

    if (currentElevation < minElevationAngle) {
        return Seconds(0);
    }

    Time currentTime = Simulator::Now();
    Time remainingTime = Seconds(0);

    for (Time t = Seconds(0); t < Seconds(1000); t += MilliSeconds(100)) {
        Time futureTime = currentTime + t;
 
        Vector futureSatPos = satMobility->GetPositionAtTime(futureTime);

        double futureElevation = CalculateElevationAngle(uePos, futureSatPos);

        if (futureElevation < minElevationAngle) {
            remainingTime = t;
            break;
        }
    }
    
    return remainingTime;
}

struct UeHandoverStatus3 
{
    Ptr<Node> servingEnb;
    Time servingSatRemainingTime;
    std::unordered_map<Ptr<Node>, uint32_t> EnbIndexMap;
};

std::map<uint32_t, UeHandoverStatus3> ueHandoverStates3;

void CheckLinkCondition_visibility(
    NodeContainer enbNodes, 
    NodeContainer ueNodes, 
    double minElevationAngle, 
    double earthRadius,
    double txPower,
    Ptr<LteHelper> lteHelper) 
{
    static std::map<uint32_t, Ptr<Node>> ueCurrentEnb;
    
    for (uint32_t ueIndex = 0; ueIndex < ueNodes.GetN(); ++ueIndex) 
    {
        Ptr<Node> ueNode = ueNodes.Get(ueIndex);
        auto& ueState = ueHandoverStates3[ueIndex];

        Ptr<Node> bestCandidate = nullptr;
        Time maxRemainingTime = Seconds(0);
        std::unordered_map<Ptr<Node>, uint32_t> EnbNodeToIndexMap;

        for (uint32_t enbIndex = 0; enbIndex < enbNodes.GetN(); ++enbIndex) 
        {
            Ptr<Node> enbNode = enbNodes.Get(enbIndex);
            EnbNodeToIndexMap[enbNode] = enbIndex;
            
            Time remainingTime = CalculateRemainingVisibleTime(ueNode, enbNode, minElevationAngle);
            
            if (remainingTime > maxRemainingTime) {
                maxRemainingTime = remainingTime;
                bestCandidate = enbNode;
            }
        }

        ueState.EnbIndexMap = EnbNodeToIndexMap;

        if (!ueCurrentEnb.count(ueIndex)) 
        { 
            Ptr<LteUeNetDevice> ueLteDev = ueNode->GetDevice(0)->GetObject<LteUeNetDevice>();
            Ptr<LteUeRrc> ueRrc = ueLteDev->GetRrc();
            uint16_t currentCellId = ueRrc->GetCellId();
            Ptr<Node> currentEnb = FindEnbByCellId(enbNodes, currentCellId);

            if (!currentEnb) {
                currentEnb = bestCandidate;
            }

            ueCurrentEnb[ueIndex] = currentEnb;
            ueState.servingEnb = currentEnb;

            Time currentRemainingTime = CalculateRemainingVisibleTime(ueNode, currentEnb, minElevationAngle);
            ueState.servingSatRemainingTime = currentRemainingTime;

            if (bestCandidate && bestCandidate != currentEnb) 
            {
                Ptr<LteEnbNetDevice> sourceEnbDev = currentEnb->GetDevice(0)->GetObject<LteEnbNetDevice>();
                Ptr<LteEnbNetDevice> targetEnbDev = bestCandidate->GetDevice(0)->GetObject<LteEnbNetDevice>();

                lteHelper->HandoverRequest(MilliSeconds(220), ueLteDev, sourceEnbDev, targetEnbDev);

                ueCurrentEnb[ueIndex] = bestCandidate;
                ueState.servingEnb = bestCandidate;
                ueState.servingSatRemainingTime = maxRemainingTime;

                std::cout << "[" << Simulator::Now().GetSeconds() << "s] UE " << ueIndex 
                          << " Initial connection to Satellite" 
                          << ueState.EnbIndexMap[bestCandidate] 
                          << " (Visible time: " << maxRemainingTime.GetSeconds() << "s)" 
                          << std::endl;
            }
            else {
                std::cout << "[" << Simulator::Now().GetSeconds() << "s] UE " << ueIndex 
                          << " Already connected to best Satellite" 
                          << ueState.EnbIndexMap[currentEnb] 
                          << " (Visible time: " << currentRemainingTime.GetSeconds() << "s)" 
                          << std::endl;
            }
        }
        else
        {
            Time currentRemainingTime = CalculateRemainingVisibleTime(ueNode, ueState.servingEnb, minElevationAngle);

            if (currentRemainingTime <= Seconds(0) )
            {
                if (!bestCandidate) continue;
                
                Ptr<LteUeNetDevice> ueLteDev = ueNode->GetDevice(0)->GetObject<LteUeNetDevice>();
                Ptr<LteEnbNetDevice> sourceEnbDev = ueState.servingEnb->GetDevice(0)->GetObject<LteEnbNetDevice>();
                Ptr<LteEnbNetDevice> targetEnbDev = bestCandidate->GetDevice(0)->GetObject<LteEnbNetDevice>();

                lteHelper->HandoverRequest(MilliSeconds(10), ueLteDev, sourceEnbDev, targetEnbDev);

                uint32_t oldIndex = ueState.EnbIndexMap[ueState.servingEnb];
                uint32_t newIndex = ueState.EnbIndexMap[bestCandidate];
                std::cout << "[" << Simulator::Now().GetSeconds() << "s] UE " << ueIndex 
                          << " HO completed: Satellite" << oldIndex 
                          << " -> Satellite" << newIndex 
                          << " (New visible time: " << maxRemainingTime.GetSeconds() << "s)" 
                          << std::endl;

                ueState.servingEnb = bestCandidate;
                ueState.servingSatRemainingTime = maxRemainingTime;
                ueCurrentEnb[ueIndex] = bestCandidate;
            }
        }
    }

    Simulator::Schedule(MilliSeconds(100), &CheckLinkCondition_visibility, enbNodes, ueNodes, 
                       minElevationAngle, earthRadius, txPower, lteHelper);
}

//-----------------------------------------------------------------------------------------------------
//--------------------------------------------main---------------------------------------------------
//-----------------------------------------------------------------------------------------------------


/**
 * Sample simulation script for LTE+EPC. It instantiates one eNodeB and several UEs,
 * attaches the UEs to the eNodeB. One remote server is set up.
 * Each UE starts a flow to and from the remote host.
 */

NS_LOG_COMPONENT_DEFINE("LteEutranEpc");

int main(int argc, char* argv[])
{
    uint16_t numUe = 1;
    uint16_t numEnb = 24;
    Time simTime = Seconds(400);
    uint32_t maxPacket = 999999999999;
    uint32_t PacketSize = 1024;//bytes
    // interPacketInterval = MicroSeconds(100);
    DataRate udpRate = DataRate("8.192Mbps");

 //卫星模型参数
    double earthRadius = 6371000.0; 
    double minElevationAngle = 37; //EAT（deg）
    double alltitudeinkm = 550;
    double inclinationindeg = 53; //（deg）
    double numorbit = 72;
    double numsatperorbit = 22;
    double FrequencyinGHz = 11.7; // ku band：10.7-12.75GHz
    double Frequency = 11.7e9;
    double txPower = 43; // 43dBm = 13dBw = 20w 113.2 
    double bandwidth = 25;
    bool closest = true;
    bool maxvisibility = false;

    // Command line arguments
    CommandLine cmd(__FILE__);
    cmd.AddValue("numUe", "Number of UE", numUe);
    cmd.AddValue("numEnb", "Number of ENB", numEnb);
    cmd.AddValue("simTime", "Time of Simulation", simTime);
    cmd.AddValue("udprate", "data rate", udpRate);
    cmd.AddValue("inclinationindeg", "inclination of the orbit", inclinationindeg);
    cmd.AddValue("alltitudeinkm", "distance(km) between the orbit and the earth surface", alltitudeinkm);
    cmd.AddValue("numorbit", "number of orbits", numorbit);
    cmd.AddValue("numsatperorbit", "number of satellites of each orbit", numsatperorbit);
    cmd.AddValue("minElevationAngle", "minimum elevation angle for user beams", minElevationAngle);
    cmd.AddValue("closest", "whether enable the closest satellite handover strategy: true=enable", closest);
    cmd.AddValue("maxvisibility", "whether enable the max visibility handover strategy: true=enable", maxvisibility);



    cmd.Parse(argc, argv);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults();

    // parse again so you can override default values from the command line
    cmd.Parse(argc, argv);
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();

    Ptr<ThreeGppNTNUrbanPropagationLossModel> lossModel = CreateObject<ThreeGppNTNUrbanPropagationLossModel>();
    lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(11.7e9));

    lteHelper->SetUeAntennaModelType ("ns3::IsotropicAntennaModel");
    lteHelper->SetUeAntennaModelAttribute ("Gain", DoubleValue (70.2));

    Config::SetDefault("ns3::LteUeRrc::T300", TimeValue(MilliSeconds(1000)));  
    Config::SetDefault("ns3::LteUeRrc::T310", TimeValue(MilliSeconds(2000000)));//2000000 对应修改的lte-ue-rrc
    Config::SetDefault("ns3::LteUeRrc::N310", UintegerValue(20000));//50000        
    Config::SetDefault("ns3::LteUeRrc::N311", UintegerValue(1));     
    
    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(txPower));//m_txPower(in dBm),60dBm=1000W
    Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode", UintegerValue(0));
    Config::SetDefault("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue(25));
                 
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numEnb);
    ueNodes.Create(numUe);


//-----------------------------------------------------------------------------------------------------
//---------------------------------------------Install Mobility Model----------------------------------
//-----------------------------------------------------------------------------------------------------
    MobilityHelper mobilityEnb;
    mobilityEnb.SetMobilityModel("ns3::SatelliteMobilityModel",
                                "Altitude", DoubleValue(alltitudeinkm),//orbit height 550 - 1050km + 6371km = 6921 - 7421km
                                "Inclination", DoubleValue(inclinationindeg),
                                "Precision", TimeValue(MilliSeconds(100)));
    Ptr<SatelliteAllocator> enbPositionAlloc = CreateObject<SatelliteAllocator>();
    enbPositionAlloc->SetAttribute("NumOrbits", IntegerValue(numorbit));
    enbPositionAlloc->SetAttribute("NumSatellites", IntegerValue(numsatperorbit));



    // ======custom configuration-shanghai,China======
    std::vector<double> customRaan_shanghai;

    customRaan_shanghai.push_back(17*M_PI/36);//85°
    customRaan_shanghai.push_back(18*M_PI/36);//90°
    customRaan_shanghai.push_back(19*M_PI/36);//95°
    customRaan_shanghai.push_back(20*M_PI/36);//100°
    customRaan_shanghai.push_back(64*M_PI/36);//320°
    customRaan_shanghai.push_back(65*M_PI/36);//325°
    customRaan_shanghai.push_back(66*M_PI/36);//330°
    customRaan_shanghai.push_back(67*M_PI/36);//335°

    std::vector<std::vector<double>> customTrueAnomaly_shanghai(8);
    for (int orbit = 0; orbit < 8; orbit++) 
    {
        if (orbit == 0) 
        {
            customTrueAnomaly_shanghai[orbit] = {-300*M_PI/792, -228*M_PI/792, -156*M_PI/792};
        } 
        else if (orbit == 1) 
        {
            customTrueAnomaly_shanghai[orbit] = {-299*M_PI/792, -227*M_PI/792, -155*M_PI/792};
        } 
        else if (orbit == 2) 
        {
            customTrueAnomaly_shanghai[orbit] = {-298*M_PI/792, -226*M_PI/792, -154*M_PI/792};
        } 
        else if (orbit == 3) 
        {
            customTrueAnomaly_shanghai[orbit] = {-297*M_PI/792, -225*M_PI/792, -153*M_PI/792};
        } 


        else if (orbit == 4)
        {
            customTrueAnomaly_shanghai[orbit] = {107*M_PI/792, 179*M_PI/792, 251*M_PI/792};
        } 
        else if (orbit == 5) 
        {
            customTrueAnomaly_shanghai[orbit] = {108*M_PI/792, 180*M_PI/792, 252*M_PI/792};
        } 
        else if (orbit == 6) 
        {
            customTrueAnomaly_shanghai[orbit] = {109*M_PI/792, 181*M_PI/792, 253*M_PI/792};
        } 
        else  
        {
            customTrueAnomaly_shanghai[orbit] = {110*M_PI/792, 182*M_PI/792, 254*M_PI/792};
        } 
    }

    // ====== custom configuration-victoriai,Seychelle======
    std::vector<double> customRaan_victoria;

    // 自定义轨道的RAAN
    customRaan_victoria.push_back(10*M_PI/36);//50°
    customRaan_victoria.push_back(11*M_PI/36);//55°
    customRaan_victoria.push_back(12*M_PI/36);//60°
    customRaan_victoria.push_back(13*M_PI/36);//65°
    customRaan_victoria.push_back(45*M_PI/36);//225°
    customRaan_victoria.push_back(46*M_PI/36);//230°
    customRaan_victoria.push_back(47*M_PI/36);//235°
    customRaan_victoria.push_back(48*M_PI/36);//240°

    std::vector<std::vector<double>> customTrueAnomaly_victoria(8);
    for (int orbit = 0; orbit < 8; orbit++) 
    {
        if (orbit == 0) 
        {
            customTrueAnomaly_victoria[orbit] = {-85*M_PI/132, -73*M_PI/132, -61*M_PI/132};
        } 
        else if (orbit == 1) 
        {
            customTrueAnomaly_victoria[orbit] = {-509*M_PI/792, -437*M_PI/792, -365*M_PI/792};
        } 
        else if (orbit == 2) 
        {
            customTrueAnomaly_victoria[orbit] = {-508*M_PI/792, -436*M_PI/792, -364*M_PI/792};
        } 
        else if (orbit == 3) 
        {
            customTrueAnomaly_victoria[orbit] = {-507*M_PI/792, -435*M_PI/792, -363*M_PI/792};
        } 


        else if (orbit == 4)
        {
            customTrueAnomaly_victoria[orbit] = {317*M_PI/792, 389*M_PI/792, 461*M_PI/792};
        } 
        else if (orbit == 5) 
        {
            customTrueAnomaly_victoria[orbit] = {318*M_PI/792, 390*M_PI/792, 462*M_PI/792};
        } 
        else if (orbit == 6) 
        {
            customTrueAnomaly_victoria[orbit] = {319*M_PI/792, 391*M_PI/792, 463*M_PI/792};
        } 
        else
        {
            customTrueAnomaly_victoria[orbit] = {320*M_PI/792, 392*M_PI/792, 464*M_PI/792};
        } 

    }

    // ======custom configuration-vancouver,Canada======
    std::vector<double> customRaan_vancouver;

    // 自定义轨道的RAAN
    customRaan_vancouver.push_back(22*M_PI/36);//g23 110°
    customRaan_vancouver.push_back(23*M_PI/36);//g24 115°
    customRaan_vancouver.push_back(24*M_PI/36);//g25 120°
    customRaan_vancouver.push_back(25*M_PI/36);//g26 125°
    customRaan_vancouver.push_back(34*M_PI/36);//g35 170°
    customRaan_vancouver.push_back(35*M_PI/36);//g36 175°
    customRaan_vancouver.push_back(36*M_PI/36);//g37 180°
    customRaan_vancouver.push_back(37*M_PI/36);//g38 185°

    std::vector<std::vector<double>> customTrueAnomaly_vancouver(8);
    for (int orbit = 0; orbit < 8; orbit++) 
    {
        if (orbit == 0) 
        {
            customTrueAnomaly_vancouver[orbit] = {-12*M_PI/792, 60*M_PI/792, 132*M_PI/792};
        } 
        else if (orbit == 1) 
        {
            customTrueAnomaly_vancouver[orbit] = {-11*M_PI/792, 61*M_PI/792, 133*M_PI/792};
        } 
        else if (orbit == 2) 
        {
            customTrueAnomaly_vancouver[orbit] = {-10*M_PI/792, 62*M_PI/792, 134*M_PI/792};
        } 
        else if (orbit == 3) 
        {
            customTrueAnomaly_vancouver[orbit] = {-9*M_PI/792, 63*M_PI/792, 135*M_PI/792};
        } 

        else if (orbit == 4)
        {
            customTrueAnomaly_vancouver[orbit] = {-144*M_PI/792, -72*M_PI/792, 0};
        } 
        else if (orbit == 5) 
        {
            customTrueAnomaly_vancouver[orbit] = {-143*M_PI/792, -71*M_PI/792, 1*M_PI/792};
        } 
        else if (orbit == 6) 
        {
            customTrueAnomaly_vancouver[orbit] = {-142*M_PI/792, -70*M_PI/792, 2*M_PI/792};
        } 
        else  
        {
            customTrueAnomaly_vancouver[orbit] = {-141*M_PI/792, -69*M_PI/792, 3*M_PI/792};
        } 
    }

    enbPositionAlloc->SetCustomParameters(customRaan_victoria, customTrueAnomaly_victoria);   
    mobilityEnb.SetPositionAllocator(enbPositionAlloc);
    mobilityEnb.Install(enbNodes);

    // Install Mobility Model
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    std::vector<Vector> customPositions = 
    {
    Vector(3614660, 5231450, -482500),          //victoria,N-4.37°，E55.27°
    // Vector(-2281086, -3504525, 4800031),     //vancouver,N49.13°，E−123.06°
    // Vector(-2847000, 4650000, 3303000),      //shanghai，N31.2303°，E121.4703°


    // Vector(3612576, 5255947, 0),             //N0°，E55.5°
    // Vector(3612486, 5255824, -55603),        //N-0.5°，E55.5°
    // Vector(3612214, 5255425, -111198),       //N-1°，E55.5°
    // Vector(3611762, 5254848, -166775),       //N-1.5°，E55.5°
    // Vector(3611131, 5254094, -222323),       //N-2°，E55.5°
    // Vector(3610320, 5253164, -277831),       //N-2.5°，E55.5°
    // Vector(3609330, 5252057, -333290),       //N-3°，E55.5°
    // Vector(3608162, 5250773, -388688),       //N-3.5°，E55.5°
    // Vector(3606816, 5249313, -444015),       //N-4°，E55.5°
    // Vector(3605292, 5247677, -499260),       //N-4.5°，E55.5°
    // Vector(3603591, 5245865, -554413),       //N-5°，E55.5°
    // Vector(3601798, 5230592, -609497),       //N-5.5°，E55.5°
    // Vector(3599727, 5241768, -664415),       //N-6°，E55.5°

    };
    for (uint16_t i = 0; i < numUe; i++) 
    {
    uint16_t index = i % customPositions.size();
    positionAlloc->Add(customPositions[index]);
    }
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(ueNodes);
        
    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

//-----------------------------------------------------------------------------------------------------
//----------------------------------------EPC and Network Configuration-----------------------------------------------
//-----------------------------------------------------------------------------------------------------
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Create a single RemoteHost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    // p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(20)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    // interface 0 is localhost, 1 is the p2p device
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    // Install the IP stack on the UEs
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

//-----------------------------------------------------------------------------------------------------
//----------------------------------------attach strategy-----------------------------------------------------
//-----------------------------------------------------------------------------------------------------

    // attach UEs to the closest gNB
    lteHelper->AttachToClosestEnb(ueLteDevs, enbLteDevs);

    // Install and start applications on UEs and remote host
    udpInterval = Time::FromDouble((PacketSize * 8) / static_cast<double>(udpRate.GetBitRate()), Time::S);

    uint16_t dlPort = 6666;
    ApplicationContainer senderApps;
    ApplicationContainer receiverApps;
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval", TimeValue(udpInterval));
        dlClient.SetAttribute("MaxPackets", UintegerValue(maxPacket));
        dlClient.SetAttribute("PacketSize", UintegerValue(PacketSize));
        senderApps.Add(dlClient.Install(remoteHost));

        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        receiverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));
    }
    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/RxWithAddresses", MakeCallback(&SinkRxWithAddress));

    senderApps.Start(MilliSeconds(30));
    receiverApps.Start(MilliSeconds(30));

    lteHelper->AddX2Interface(enbNodes);

//-----------------------------------------------------------------------------------------------------
//----------------------------------------Output-----------------------------------------------------
//-----------------------------------------------------------------------------------------------------

    // creat FlowMonitor
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowMonitor = flowHelper.Install(remoteHost);
    flowMonitor = flowHelper.Install(ueNodes);

    double monitoringInterval = 1.0; //sec
    bool firstTime = true;
    std::map<FlowId, uint64_t> lastRxPackets;
    std::map<FlowId, uint64_t> lastTxPackets;
    std::map<FlowId, uint64_t> lastLostPackets;

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    Simulator::Schedule(Seconds(monitoringInterval), &CheckFlowStats, flowMonitor, classifier, 
                       monitoringInterval, std::ref(firstTime), std::ref(lastRxPackets), std::ref(lastTxPackets), std::ref(lastLostPackets));


    // connect custom trace sinks for RRC connection establishment and handover notification
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
        MakeCallback(&NotifyConnectionEstablishedEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
            MakeCallback(&NotifyConnectionEstablishedUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
            MakeCallback(&NotifyHandoverStartEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
            MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
            MakeCallback(&NotifyHandoverEndOkEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
            MakeCallback(&NotifyHandoverEndOkUe));

    // Hook a trace sink (the same one) to the four handover failure traces
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureNoPreamble",
            MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureMaxRach",
            MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureLeaving",
            MakeCallback(&NotifyHandoverFailure));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverFailureJoining",
            MakeCallback(&NotifyHandoverFailure));


    Simulator::Schedule(Seconds(0.0), &PrintAllUePositions, ueNodes);//UE Position
    Simulator::Schedule(Seconds(0.0), &PrintAllEnbPositions, enbNodes);//SAT Position
    Simulator::Schedule(Seconds(0.0), &PrintCutoffDistance, enbNodes, minElevationAngle); //Cutoff Distance
    Simulator::Schedule(Seconds(0.0), &GetCutoffDistance, enbNodes, ueNodes, minElevationAngle, txPower);//EAT

    if (closest)
    {
        //Cloest-satellite HO
        Simulator::Schedule(Seconds(0.0), &CheckLinkCondition_closest, enbNodes, ueNodes, minElevationAngle, earthRadius, txPower, lteHelper);//CHO
    }
    
    if (maxvisibility)
    {
        //Max-visibility HO
        Simulator::Schedule(Seconds(0.0), &CheckLinkCondition_visibility, enbNodes, ueNodes, minElevationAngle, earthRadius, txPower, lteHelper);//CHO
    }
   
    Simulator::Stop(simTime);
    Simulator::Run();

    flowMonitor->CheckForLostPackets();
    std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();
    
    std::cout << "\n===== FINAL STATISTICS =====" << std::endl;
    for (auto iter = stats.begin(); iter != stats.end(); ++iter)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(iter->first);
        double lossRate = (iter->second.txPackets > 0) ? 
                         (static_cast<double>(iter->second.lostPackets) / iter->second.txPackets * 100.0) : 0.0;
        
        std::cout << "Flow " << iter->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")" << std::endl;
        std::cout << "  Total Tx Packets: " << iter->second.txPackets << std::endl;
        std::cout << "  Total Rx Packets: " << iter->second.rxPackets << std::endl;
        std::cout << "  Total Lost Packets: " << iter->second.lostPackets << std::endl;
        std::cout << "  Total Loss Rate: " << lossRate << "%" << std::endl;
    }

    Simulator::Destroy();
    return 0;
}


