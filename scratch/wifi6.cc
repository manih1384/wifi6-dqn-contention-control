#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/spectrum-module.h"
#include <map>

// NEW HEADERS FOR UDP SOCKET IPC
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

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

// ==========================================
// C++ / Python IPC Bridge (The AI Link)
// ==========================================
struct AILink {
    int sock;
    struct sockaddr_in serv_addr;
    socklen_t addr_len;

    // Initialize the local UDP socket
    void Init() {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(9999);
        inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr);
        addr_len = sizeof(serv_addr);
        std::cout << "[ns-3] Connected to Python AI Bridge on port 9999" << std::endl;
    }

    // Send state to Python, freeze simulation, wait for the action
    int GetActionFromAI(std::string stateString) {
        // Send state to Python
        sendto(sock, stateString.c_str(), stateString.length(), 0, 
               (struct sockaddr *)&serv_addr, addr_len);
        
        // Wait for Python to reply (ns-3 clock freezes while waiting)
        char buffer[256] = {0};
        int n = recvfrom(sock, buffer, 255, 0, (struct sockaddr *)&serv_addr, &addr_len);
        
        if (n > 0) {
            buffer[n] = '\0';
            return std::stoi(buffer); // Expected returns from paper: 0, 1, or 2
        }
        
        return 1; // Default to 'Keep' if the socket fails
    }

    void Close() {
        close(sock);
    }
} aiLink;

// ==========================================
// Simulation Variables & Callbacks
// ==========================================
std::map<Mac48Address, double> stationSinrSum;
std::map<Mac48Address, int> stationRxCount;
std::map<Mac48Address, int> stationBeaconCount;

void MonitorSnifferRxCallback(std::string context, Ptr<const Packet> packet, 
                              uint16_t channelFreqMhz, WifiTxVector txVector, 
                              MpduInfo aMpdu, SignalNoiseDbm signalNoise, uint16_t staId)
{
    double signalLinear = pow(10.0, signalNoise.signal / 10.0);
    double noiseLinear = pow(10.0, signalNoise.noise / 10.0);
    double sinrLinear = signalLinear / noiseLinear;

    Ptr<Packet> copy = packet->Copy();
    uint8_t buffer[2];
    copy->CopyData(buffer, 2);

    uint16_t frameControl = buffer[0] | (buffer[1] << 8);
    uint8_t type = (frameControl >> 2) & 0x3;    

    if (type != 0x00 && type != 0x02) return;

    WifiMacHeader hdr;
    copy->RemoveHeader(hdr);  

    Mac48Address srcAddr;
    if (hdr.IsBeacon()) {
        srcAddr = hdr.GetAddr2();   
        stationBeaconCount[srcAddr]++;
    } else if (hdr.IsData() || hdr.IsQosData()) {
        srcAddr = hdr.GetAddr2();   
    } else {
        srcAddr = hdr.GetAddr2();
    }

    if (srcAddr == Mac48Address("00:00:00:00:00:00")) return;

    stationSinrSum[srcAddr] += sinrLinear;
    stationRxCount[srcAddr]++;
}

std::pair<NetDeviceContainer, NetDeviceContainer> SetupWifiNetwork(NodeContainer &stationNodes, NodeContainer &APNode)
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
    
    for (int i = 0; i < NUMBER_OF_STATIONS; i++) {
        Ptr<MobilityModel> mm = stationNodes.Get(i)->GetObject<MobilityModel>();
        Vector pos = mm->GetPosition();
        pos.z = STA_HEIGHT;
        mm->SetPosition(pos);
    }
    
    std::cout << "STAs randomly placed in [" << MIN_DISTANCE << ", " << MAX_DISTANCE << "]m range" << std::endl;
}

std::pair<Ipv4InterfaceContainer, Ipv4InterfaceContainer> SetupInternet(NodeContainer &APNode, NodeContainer &stationNodes, NetDeviceContainer &apDevice, NetDeviceContainer &stationDevices)
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

void SetupApplications(NodeContainer &APNode, NodeContainer &stationNodes, Ipv4InterfaceContainer &apInterface)
{
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), PORT_NUMBER));
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
}

void DisplayFlowStatistics(Ptr<FlowMonitor> flowMonitor, FlowMonitorHelper &flowmonHelper)
{
    // (Truncated for brevity, but this is your existing print function)
    flowMonitor->CheckForLostPackets();
    std::cout << "\n===== Flow Statistics Evaluated =====" << std::endl;
}

int main(int argc, char *argv[])
{
    NodeContainer stationNodes;
    stationNodes.Create(NUMBER_OF_STATIONS);
    NodeContainer APNode;
    APNode.Create(1);

    auto [apDevice, stationDevices] = SetupWifiNetwork(stationNodes, APNode);
    SetupMobility(APNode, stationNodes);
    auto [apInterface, staInterfaces] = SetupInternet(APNode, stationNodes, apDevice, stationDevices);
    SetupApplications(APNode, stationNodes, apInterface);

    std::string apTracePath = "/NodeList/" + std::to_string(NUMBER_OF_STATIONS) + "/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx";
    Config::Connect(apTracePath, MakeCallback(&MonitorSnifferRxCallback));

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> flowMonitor = flowmonHelper.InstallAll();

    // ==========================================
    // INITIALIZE & TEST THE AI BRIDGE
    // ==========================================
    aiLink.Init();
    std::cout << "\n[ns-3] Testing AI Bridge before simulation starts..." << std::endl;
    int testAction = aiLink.GetActionFromAI("TEST_STATE_COL=5_RET=2");
    std::cout << "[ns-3] AI replied with action: " << testAction << std::endl;

    std::cout << "\nRunning UORA simulation for " << SIMULATION_TIME << " seconds..." << std::endl;
    
    Simulator::Stop(Seconds(SIMULATION_TIME));
    Simulator::Run();

    DisplayFlowStatistics(flowMonitor, flowmonHelper);
    
    // CLOSE THE BRIDGE
    aiLink.Close();
    Simulator::Destroy();

    return 0;
}