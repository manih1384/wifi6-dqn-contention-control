#include "ns3/core-module.h"

using namespace ns3;

int main(int argc, char* argv[])
{
    std::cout << "Hello from ns-3!" << std::endl;
    std::cout << "Build is working correctly!" << std::endl;
    
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
