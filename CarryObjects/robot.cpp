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


#include "robot.h"

#include <iostream>

const double dstX = 5.0;
const double dstY = 5.0;

const double srcX = -5.2;
const double srcY = -5.2;

static const double cruiseSpeed = 0.5;
static const double cruiseTurn = 1.0;
static const double avoidSpeed = 0.2;
static const double avoidTurn = 1.0;
static const double stopDist = 0.6;
static const int maxAvoidDelay = 1;

double have[4][4] = {
    { 90, 180, 180, 180 },
    { 90, -90, 180, 90 },
    { 80, 90, 45, 90 },
    { 20, 45, 0, 0}
};

double need[4][4] = {
    { -120, -180, 180, 180 },
    { -90, -120, 180, 90 },
    { -90, -135, 180, 180 },
    { -90, -180, -90, -135 }
};

inline double sgn(double x) {
    return x > 0 ? 1 : (x < 0 ? -1 : 0);
}

inline double normAngle(double angle) {
    return std::fmod(angle + 2 * M_PI, 2 * M_PI);
}

inline double angleDifference(double a, double b) {
    double diff = a - b;

    int num2Pi = sgn(diff) * (fabs(diff) / (2 * M_PI) + 0.5);
    return diff - num2Pi * 2 * M_PI;
}

inline double pow2(double x) {
    return x * x;
}

Robot::Robot(PlayerCc::PlayerClient &client, int id)
        : m_id(id),
        m_client(client),
        m_pos(&client, 0),
        m_ranger(&client, 0),
        m_simulation(&client, 0),
        m_object(0),
        m_avoidDelay(0)
{
    m_pos.RequestGeom();
    m_ranger.RequestGeom();

    // Read one more time to avoid SIGSEGV in RangerProxy
    m_client.Read();

    int rangeCount = m_ranger.GetRangeCount();
    std::cout << "RangeCount: " << rangeCount << std::endl;
}

Object* Robot::findObject(double searchDistance)
{
    const int numObjects = 4;

    double minDistance = 1e8;
    Object resultObject;
    bool haveResult = false;

    double bobX = m_pos.GetXPos();
    double bobY = m_pos.GetYPos();
    double bobYaw = m_pos.GetYaw();


    for (int i = 1; i <= numObjects; i++) {
        std::stringstream stream;
        stream << "ghost" << i;
        std::string name = stream.str();

        double objX, objY, objYaw;
        m_simulation.GetPose2d(const_cast<char*>(name.c_str()), objX, objY, objYaw);

        double distance = pow2(objX - bobX) + pow2(objY - bobY);
        double yaw = angleDifference(normAngle(std::atan2(objY - bobY, objX - bobX)), normAngle(bobYaw));

        if (distance < pow2(searchDistance) &&
                std::abs(yaw) < M_PI_2 &&
                distance < minDistance) {

            std::cout << "Found an object: " << distance << " " << name << std::endl;

            haveResult = true;

            resultObject.name = name;
            resultObject.x = objX;
            resultObject.y = objY;
            resultObject.yaw = objYaw;
        }
    }

    return haveResult ? new Object(resultObject) : 0;
}

void Robot::doIteration()
{
    double xSpeed = cruiseSpeed;
    double yawSpeed = cruiseTurn;

    if (m_object) {
        goSomewhere(have, xSpeed, yawSpeed);
        tryDrop();
    }
    else {
        goSomewhere(need, xSpeed, yawSpeed);
        tryCatch();
    }

    avoidObstacle(xSpeed, yawSpeed);

    m_pos.SetSpeed(xSpeed, yawSpeed);
}

void Robot::goSomewhere(double gradient[4][4], double &xSpeed, double &yawSpeed)
{
    double bobX = m_pos.GetXPos();
    double bobY = m_pos.GetYPos();
    double bobYaw = m_pos.GetYaw();

    int x = (bobX + 6) / 3;
    int y = (bobY + 6) / 3;

    if ( x > 3 ) x = 3;
    if ( y > 3 ) y = 3;
    if ( x < 0 ) x = 0;
    if ( y < 0 ) y = 0;

    double yawGoal = PlayerCc::dtor(gradient[y][x]);

    goToAngle(yawGoal, xSpeed, yawSpeed);
}

void Robot::tryCatch()
{
    const double catchDistance = 0.5;

    Object *obj = findObject(catchDistance);
    if (obj) {
        if (obj->x == dstX && obj->y == dstY) {
            delete obj;
            return;
        }

        m_object = obj;
        m_simulation.SetPose2d(const_cast<char*>(m_object->name.c_str()), -100, -100, 0);
    }
}

void Robot::tryDrop()
{
    const double dropDistance = 0.5;

    if (m_object) {
        double bobX = m_pos.GetXPos();
        double bobY = m_pos.GetYPos();

        if (pow2(bobX - dstX) + pow2(bobY - dstY) < pow2(dropDistance)) {
            m_simulation.SetPose2d(const_cast<char*>(m_object->name.c_str()), dstX, dstY, 0);
            delete m_object;
            m_object = 0;
        }
    }
}

void Robot::goToAngle(double dstYaw, double &xSpeed, double &yawSpeed)
{
    double bobYaw = m_pos.GetYaw();

    double yaw = angleDifference(normAngle(dstYaw), normAngle(bobYaw));

    yawSpeed = std::max(1.0, std::abs(yaw) / M_PI * 10) * sgn(yaw) * cruiseTurn;
}

void Robot::goToPoint(double x, double y, double &xSpeed, double &yawSpeed)
{
    double bobX = m_pos.GetXPos();
    double bobY = m_pos.GetYPos();
    double bobYaw = m_pos.GetYaw();

    double yaw = angleDifference(normAngle(std::atan2(x - bobY, y - bobX)), normAngle(bobYaw));

    yawSpeed = std::max(1.0, std::abs(yaw) / M_PI * 10) * sgn(yaw) * cruiseTurn;
}

void Robot::avoidObstacle(double &xSpeed, double &yawSpeed)
{
    bool stop = false;

    // find the closest distance to the left and right and check if
    // there's anything in front
    double minLeft = 1e6;
    double minRight = 1e6;

    int sampleCount = m_ranger.GetRangeCount();

    for (uint32_t i = 0; i < sampleCount; i++) {

        stop |= (i > (sampleCount/2.5))
                && (i < (sampleCount - (sampleCount/2.5)))
                && m_ranger[i] < stopDist;


        if ( i > sampleCount/2 )
            minLeft = std::min(minLeft, m_ranger[i]);
        else
            minRight = std::min(minRight, m_ranger[i]);

        if (stop && m_avoidDelay < 1) {
            m_avoidDelay = random() % maxAvoidDelay + 1;
        }
    }

    if (stop || m_avoidDelay > 0) {
        xSpeed = stop ? 0.0 : avoidSpeed;

        double turnCoeff = minLeft < minRight ? -1 : 1;
        yawSpeed = turnCoeff * avoidTurn;

        m_avoidDelay--;
    }
}
