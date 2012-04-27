#include <iostream>
#include <unistd.h>
#include <libplayerc++/playerc++.h>

int main(int argc, char **argv) {
    PlayerCc::PlayerClient robot("localhost", 6665);
    PlayerCc::Position2dProxy pos(&robot, 0);
    PlayerCc::RangerProxy ranger(&robot, 1);
  
    pos.RequestGeom();
    ranger.RequestGeom();
    
    // Read one more time to avoid SIGSEGV in RangerProxy
    robot.Read();
    
    int rangeCount = ranger.GetRangeCount();
    std::cout << "RangeCount: " << rangeCount << std::endl;
    
    while(1) {
        robot.Read();

        std::cout << "Position:  X: " << pos.GetXPos() << "\tY: " << pos.GetYPos() << "\tYaw: " << PlayerCc::rtod(pos.GetYaw()) << std::endl;
        std::cout << "Distances: 0: " << ranger[0] 
        << "\t90: " << ranger[rangeCount / 2] 
        << "\t180: " << ranger[rangeCount - 1] << std::endl;
        std::cout << std::endl;

        usleep(500000);
    }
    
    return 0;
}
