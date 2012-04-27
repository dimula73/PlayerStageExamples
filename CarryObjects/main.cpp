#include <iostream>
#include <libplayerc++/playerc++.h>

#include "robot.h"

int main(int argc, char **argv) {
    PlayerCc::PlayerClient client("localhost", 6665);
    
    Robot robot(client, 0);
     
    while(1) {
        client.Read();
        robot.doIteration();
    }
    
    return 0;
}
