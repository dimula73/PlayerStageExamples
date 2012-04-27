/*
    Copyright (C) 2012  Dmitry Kazakov dimula73@gmail.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/


#ifndef ROBOT_H
#define ROBOT_H

#include <libplayerc++/playerc++.h>

#include <string>
#include <sstream>

struct Object {
    double x;
    double y;
    double yaw;
    std::string name;
};

/**
 * A simple class that emulates a robot carrying objects
 * from one side of the room to another
 */

class Robot
{
        
public:
    Robot(PlayerCc::PlayerClient &client, int id);
    
    void doIteration();
    
private:
    void goSomewhere(double gradient[4][4], double &xSpeed, double &yawSpeed);
       
    void tryCatch();
    void tryDrop();
    
    Object* findObject(double searchDistance);
    void avoidObstacle(double &xSpeed, double &yawSpeed);
    void goToAngle(double dstYaw, double &xSpeed, double &yawSpeed);
    void goToPoint(double x, double y, double &xSpeed, double &yawSpeed);
    
private:
    int m_id;
    PlayerCc::PlayerClient &m_client;
    
    PlayerCc::Position2dProxy m_pos;
    PlayerCc::RangerProxy m_ranger;
    PlayerCc::SimulationProxy m_simulation;
    
    Object *m_object;
    
    int m_avoidDelay;
};

#endif // ROBOT_H
