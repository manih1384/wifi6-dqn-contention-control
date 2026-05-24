#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/spectrum-module.h"
#include <map>

using namespace ns3;

const int NUMBER_OF_STATIONS = 40;
const double MIN_DISTANCE = 5.0;
const double MAX_DISTANCE = 30.0;
const double AP_HEIGHT = 3.0;
const double STA_HEIGHT = 1.0;
const uint16_t PORT_NUMBER = 9;
const double SIMULATION_TIME = 10.0;
const double PACKET_INTERVAL = 0.005;  // 5 ms
const int PACKET_SIZE = 93;            // bytes
const std::string SSID_NAME = "wifi6-uora-network";
const std::string IP_BASE = "192.168.1.0";
const std::string IP_MASK = "255.255.255.0";

// Data structures for SINR tracking
std::map<Mac48Address, double> stationSinrSum;
std::map<Mac48Address, int> stationRxCount;
std::map<Mac48Address, int> stationBeaconCount;

void MonitorSnifferRxCallback(std::string context, 
                              Ptr<const Packet> packet, 
                              uint16_t channelFreqMhz, 
                              WifiTxVector txVector, 
                              MpduInfo aMpdu, 
                              SignalNoiseDbm signalNoise,
                              uint16_t staId)
{
    // Calculate linear SINR 
    double signalLinear = pow(10.0, signalNoise.signal / 10.0);
    double noiseLinear = pow(10.0, signalNoise.noise / 10.0);
    double sinrLinear = signalLinear / noiseLinear;

    // Manually check frame type to avoid NS_ASSERT on unknown subtypes
    // Frame control is the first 2 bytes of the MAC header.
    Ptr<Packet> copy = packet->Copy();
    uint8_t buffer[2];
    copy->CopyData(buffer, 2);

    // Frame control fields 
    uint16_t frameControl = buffer[0] | (buffer[1] << 8);
    uint8_t type = (frameControl >> 2) & 0x3;    // bits 2-3

    // Only process Data and Management frames (ignore Control and Extension)
    if (type != 0x00 && type != 0x02)   // 0=Management, 2=Data
        return;

    // Now it's safe to parse the full header
    WifiMacHeader hdr;
    copy->RemoveHeader(hdr);  // copy already advanced past the first 2 bytes

    // Determine source MAC
    Mac48Address srcAddr;
    if (hdr.IsBeacon()) {
        srcAddr = hdr.GetAddr2();   // AP's address
        stationBeaconCount[srcAddr]++;
    } else if (hdr.IsData() || hdr.IsQosData()) {
        srcAddr = hdr.GetAddr2();   // transmitter (STA or AP)
    } else {
        // other management frames (probe, assoc, etc.)
        srcAddr = hdr.GetAddr2();
    }

    if (srcAddr == Mac48Address("00:00:00:00:00:00"))
        return;

    // Accumulate
    stationSinrSum[srcAddr] += sinrLinear;
    stationRxCount[srcAddr]++;

    // Occasional debug
    if (stationRxCount[srcAddr] % 500 == 0) {
        double avgSinrDb = 10.0 * log10(stationSinrSum[srcAddr] / stationRxCount[srcAddr]);
        std::cout << "STA " << srcAddr << " | Pkts: " << stationRxCount[srcAddr]
                  << " | Avg SINR: " << avgSinrDb << " dB | STA-ID: " << staId << std::endl;
    }
}

std::pair<NetDeviceContainer, NetDeviceContainer>
SetupWifiNetwork(NodeContainer &stationNodes, NodeContainer &APNode)
{
    SpectrumWifiPhyHelper phy;
    SpectrumChannelHelper channelHelper = SpectrumChannelHelper::Default();
    channelHelper.SetChannel("ns3::MultiModelSpectrumChannel");
    phy.SetChannel(channelHelper.Create());

    phy.Set("ChannelSettings", StringValue("{38, 40, BAND_5GHZ, 0}"));
    phy.Set("TxPowerStart", DoubleValue(16.0));
    phy.Set("TxPowerEnd", DoubleValue(16.0));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ax);
    
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("HeMcs5"),
                                 "ControlMode", StringValue("HeMcs2"));

    WifiMacHelper mac;
    Ssid ssid = Ssid(SSID_NAME);

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    
    mac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
                              "EnableUlOfdma", BooleanValue(true),
                              "EnableBsrp", BooleanValue(true),
                              "UseCentral26TonesRus", BooleanValue(true),
                              "NStations", UintegerValue(8));

    NetDeviceContainer apDevice = wifi.Install(phy, mac, APNode);

    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer stationDevices = wifi.Install(phy, mac, stationNodes);

    std::cout << "Installed Wi-Fi 6 (802.11ax) with OFDMA + BSRP + UORA" << std::endl;

    return std::make_pair(apDevice, stationDevices);
}

void SetupMobility(NodeContainer &APNode, NodeContainer &stationNodes)
{
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<ListPositionAllocator> apAlloc = CreateObject<ListPositionAllocator>();
    apAlloc->Add(Vector(0.0, 0.0, AP_HEIGHT));
    mobility.SetPositionAllocator(apAlloc);
    mobility.Install(APNode);

    // Create separate MobilityHelper for stations to avoid conflicts
    MobilityHelper staMobility;
    staMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    Ptr<RandomDiscPositionAllocator> staAlloc = CreateObject<RandomDiscPositionAllocator>();
    staAlloc->SetX(0.0);
    staAlloc->SetY(0.0);
    staAlloc->SetRho(CreateObjectWithAttributes<UniformRandomVariable>(
        "Min", DoubleValue(MIN_DISTANCE), 
        "Max", DoubleValue(MAX_DISTANCE)));
    staMobility.SetPositionAllocator(staAlloc);
    staMobility.Install(stationNodes);
    
    // Enforce station heights
    for (int i = 0; i < NUMBER_OF_STATIONS; i++) {
        Ptr<MobilityModel> mm = stationNodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mm->GetPosition();
        pos.z = STA_HEIGHT;
        mm->SetPosition(pos);
    }
    
    std::cout << "STAs randomly placed in [" << MIN_DISTANCE 
              << ", " << MAX_DISTANCE << "]m range" << std::endl;
}

std::pair<Ipv4InterfaceContainer, Ipv4InterfaceContainer>
SetupInternet(NodeContainer &APNode, NodeContainer &stationNodes,
              NetDeviceContainer &apDevice, NetDeviceContainer &stationDevices)
{
    InternetStackHelper stack;
    stack.Install(APNode);
    stack.Install(stationNodes);

    Ipv4AddressHelper address;
    address.SetBase(IP_BASE.c_str(), IP_MASK.c_str());

    Ipv4InterfaceContainer apInterface = address.Assign(apDevice);
    Ipv4InterfaceContainer staInterfaces = address.Assign(stationDevices);

    std::cout << "AP IP: " << apInterface.GetAddress(0) << std::endl;
    
    return std::make_pair(apInterface, staInterfaces);
}

void SetupApplications(NodeContainer &APNode, NodeContainer &stationNodes, 
                       Ipv4InterfaceContainer &apInterface)
{
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", 
                                InetSocketAddress(Ipv4Address::GetAny(), PORT_NUMBER));
    ApplicationContainer sinkApp = sinkHelper.Install(APNode.Get(0));
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(SIMULATION_TIME));

    Ptr<UniformRandomVariable> randomStart = CreateObject<UniformRandomVariable>();
    randomStart->SetAttribute("Min", DoubleValue(0.0));
    randomStart->SetAttribute("Max", DoubleValue(0.05));

    for (int i = 0; i < NUMBER_OF_STATIONS; i++)
    {
        UdpClientHelper client(apInterface.GetAddress(0), PORT_NUMBER);
        client.SetAttribute("PacketSize", UintegerValue(PACKET_SIZE));
        client.SetAttribute("Interval", TimeValue(Seconds(PACKET_INTERVAL)));
        client.SetAttribute("MaxPackets", UintegerValue(100000));

        ApplicationContainer clientApp = client.Install(stationNodes.Get(i));
        double start = randomStart->GetValue();
        clientApp.Start(Seconds(start));
        clientApp.Stop(Seconds(SIMULATION_TIME));
    }
    
    std::cout << "Traffic: " << PACKET_SIZE << "B packets, " 
              << PACKET_INTERVAL * 1000 << "ms interval" << std::endl;
}

void DisplayFlowStatistics(Ptr<FlowMonitor> flowMonitor, FlowMonitorHelper &flowmonHelper)
{
    flowMonitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = flowMonitor->GetFlowStats();

    double totalThroughput = 0.0, totalDelay = 0.0;
    double totalTx = 0.0, totalRx = 0.0, totalLost = 0.0;
    int numFlows = 0;
    std::vector<double> throughputs;

    std::cout << "\n===== UORA (802.11ax) Per-Node Flow Statistics =====" << std::endl;
    
    for (auto &flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        // Calculate true application-level loss
        double txPackets = flow.second.txPackets;
        double rxPackets = flow.second.rxPackets;
        double trueLostPackets = txPackets - rxPackets;
        
        double duration = flow.second.timeLastRxPacket.GetSeconds() - 
                         flow.second.timeFirstTxPacket.GetSeconds();
        double throughput = (duration > 0 && rxPackets > 0) ? 
                           flow.second.rxBytes * 8.0 / duration / 1000.0 : 0.0;
        double avgDelay = (rxPackets > 0) ? 
                         flow.second.delaySum.GetSeconds() / rxPackets : 0.0;
        double lossPercent = (txPackets > 0) ? 
                            (trueLostPackets / txPackets) * 100.0 : 0.0;

        std::cout << "Flow " << numFlows + 1 << ": " << t.sourceAddress << " -> " << t.destinationAddress << std::endl;
        std::cout << "  Tx: " << txPackets << "  Rx: " << rxPackets 
                  << "  Lost: " << trueLostPackets << " (" << lossPercent << "%)" << std::endl;
        std::cout << "  Throughput: " << throughput << " kbps  Avg Delay: " << avgDelay * 1000.0 << " ms\n" << std::endl;

        throughputs.push_back(throughput);
        totalThroughput += throughput;
        totalDelay += avgDelay;
        totalTx += txPackets;
        totalRx += rxPackets;
        totalLost += trueLostPackets;
        numFlows++;
    }

    double sum = 0.0, sumSq = 0.0;
    for (double t : throughputs) {
        sum += t;
        sumSq += t * t;
    }
    double jainIndex = (numFlows > 0 && sumSq > 0) ? (sum * sum) / (numFlows * sumSq) : 0.0;

    double avgThroughput = totalThroughput / numFlows;
    double avgDelay = totalDelay / numFlows;
    double overallLoss = (totalTx > 0) ? (totalLost / totalTx) * 100.0 : 0.0;

    std::cout << "=================================================" << std::endl;
    std::cout << "SUMMARY" << std::endl;
    std::cout << "=================================================" << std::endl;
    std::cout << "Flows: " << numFlows << std::endl;
    std::cout << "Total Tx: " << totalTx << " | Total Rx: " << totalRx 
              << " | Lost: " << totalLost << " (" << overallLoss << "%)" << std::endl;
    std::cout << "Average Throughput: " << avgThroughput << " kbps" << std::endl;
    std::cout << "Total Throughput: " << totalThroughput << " kbps" << std::endl;
    std::cout << "Average Delay: " << avgDelay * 1000.0 << " ms" << std::endl;
    std::cout << "Jain's Fairness Index: " << jainIndex << std::endl;
    
    std::cout << "\n===== AP Received SINR Report =====" << std::endl;
    for (auto& entry : stationSinrSum) {
        // Skip the phantom A-MPDU MAC address 
        if (entry.first == Mac48Address("00:00:00:01:00:00")) continue;

        if (stationRxCount[entry.first] > 0) {
            double avgSinrDb = 10.0 * log10(entry.second / stationRxCount[entry.first]);
            std::cout << "STA MAC " << entry.first 
                      << " | Rx PHY Bursts: " << stationRxCount[entry.first]
                      << " | Avg SINR: " << avgSinrDb << " dB" << std::endl;
        }
    }
}



int main(int argc, char *argv[])
{
    NodeContainer stationNodes;
    stationNodes.Create(NUMBER_OF_STATIONS);
    NodeContainer APNode;
    APNode.Create(1);

    auto [apDevice, stationDevices] = SetupWifiNetwork(stationNodes, APNode);
    SetupMobility(APNode, stationNodes);
    auto [apInterface, staInterfaces] = SetupInternet(APNode, stationNodes, 
                                                       apDevice, stationDevices);
    SetupApplications(APNode, stationNodes, apInterface);

    std::string apTracePath = "/NodeList/" + std::to_string(NUMBER_OF_STATIONS) + "/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
    Config::Connect(apTracePath, MakeCallback(&MonitorSnifferRxCallback));

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> flowMonitor = flowmonHelper.InstallAll();

    std::cout << "\nRunning UORA simulation for " << SIMULATION_TIME << " seconds..." << std::endl;
    
    Simulator::Stop(Seconds(SIMULATION_TIME));
    Simulator::Run();

    DisplayFlowStatistics(flowMonitor, flowmonHelper);
    Simulator::Destroy();

    return 0;
}