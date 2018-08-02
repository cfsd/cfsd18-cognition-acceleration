/**
* Copyright (C) 2017 Chalmers Revere
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
* USA.
*/

#include <iostream>
#include <cmath>
#include "acceleration.hpp"
#include <chrono>

Acceleration::Acceleration(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4) :
  m_od4(od4),
  m_groundSpeed{0.0f},
  m_groundSpeedMutex{},
  m_tickDt{},
  m_tockDt{},
  m_steerTickDt{},
  m_steerTockDt{},
  m_ei{0.0f},
  m_ePrev{0.0f},
  m_changeState{true},
  m_prevHeadingRequest{0.0f},
  m_accelerate{true},
  m_STOP{false},
  m_latestPathSet{false},
  m_latestPath{},
  m_distanceTraveled{0.0f},
  m_groundSpeedReadingLeft{0.0f},
  m_groundSpeedReadingRight{0.0f},
  m_sendMutex()
{
 setUp(commandlineArguments);
 m_tickDt = std::chrono::system_clock::now();
 m_steerTickDt = std::chrono::system_clock::now();
}
Acceleration::~Acceleration()
{
}
void Acceleration::setUp(std::map<std::string, std::string> commandlineArguments)
{
  m_speedId1=(commandlineArguments["speedId1"].size() != 0) ? (static_cast<uint32_t>(std::stoi(commandlineArguments["speedId1"]))) : (m_speedId1);
  m_speedId2=(commandlineArguments["speedId2"].size() != 0) ? (static_cast<uint32_t>(std::stoi(commandlineArguments["speedId2"]))) : (m_speedId2);
  m_senderStamp=(commandlineArguments["id"].size() != 0) ? (static_cast<int>(std::stoi(commandlineArguments["id"]))) : (m_senderStamp);
  // steering
  m_usePathMemory=(commandlineArguments["usePathMemory"].size() != 0) ? (std::stoi(commandlineArguments["usePathMemory"])==1) : (true);
  m_useAimDistanceLapCounter=(commandlineArguments["useAimDistanceLapCounter"].size() != 0) ? (std::stoi(commandlineArguments["useAimDistanceLapCounter"])==1) : (true);
  m_staticTrustInLastPathPoint=(commandlineArguments["staticTrustInLastPathPoint"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["staticTrustInLastPathPoint"]))) : (m_staticTrustInLastPathPoint);
  m_useDynamicTrust=(commandlineArguments["useDynamicTrust"].size() != 0) ? (std::stoi(commandlineArguments["useDynamicTrust"])==1) : (false);
  m_lowTrustLimDistance=(commandlineArguments["lowTrustLimDistance"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["lowTrustLimDistance"]))) : (m_lowTrustLimDistance);
  m_highTrustLimDistance=(commandlineArguments["highTrustLimDistance"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["highTrustLimDistance"]))) : (m_highTrustLimDistance);
  m_lowTrustLim=(commandlineArguments["lowTrustLim"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["lowTrustLim"]))) : (m_lowTrustLim);
  m_highTrustLim=(commandlineArguments["highTrustLim"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["highTrustLim"]))) : (m_highTrustLim);
  m_aimDistance=(commandlineArguments["aimDistance"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aimDistance"]))) : (m_aimDistance);
  m_moveOrigin=(commandlineArguments["useMoveOrigin"].size() != 0) ? (std::stoi(commandlineArguments["useMoveOrigin"])==1) : (true);
  m_steerRate=(commandlineArguments["steerRate"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["steerRate"]))) : (m_steerRate);
  m_prevReqRatio=(commandlineArguments["prevReqRatio"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["prevReqRatio"]))) : (m_prevReqRatio);
  // velocity control
  m_velocityLimit=(commandlineArguments["velocityLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["velocityLimit"]))) : (m_velocityLimit);
  m_axLimitPositive=(commandlineArguments["axLimitPositive"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axLimitPositive"]))) : (m_axLimitPositive);
  m_axLimitNegative=(commandlineArguments["axLimitNegative"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["axLimitNegative"]))) : (m_axLimitNegative);
  // ....controller
  m_aKp=(commandlineArguments["aKp"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aKp"]))) : (m_aKp);
  m_aKd=(commandlineArguments["aKd"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aKd"]))) : (m_aKd);
  m_aKi=(commandlineArguments["aKi"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["aKi"]))) : (m_aKi);
  m_bKp=(commandlineArguments["bKp"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["bKp"]))) : (m_bKp);
  m_bKd=(commandlineArguments["bKd"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["bKd"]))) : (m_bKd);
  m_bKi=(commandlineArguments["bKi"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["bKi"]))) : (m_bKi);

  // vehicle specific
  m_wheelAngleLimit=(commandlineArguments["wheelAngleLimit"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["wheelAngleLimit"]))) : (m_wheelAngleLimit);
  m_frontToCog=(commandlineArguments["frontToCog"].size() != 0) ? (static_cast<float>(std::stof(commandlineArguments["frontToCog"]))) : (m_frontToCog);

  /*//std::cout<<"Acceleration set up with "<<commandlineArguments.size()<<" commandlineArguments: "<<std::endl;
  for (std::map<std::string, std::string >::iterator it = commandlineArguments.begin();it !=commandlineArguments.end();it++){
    //std::cout<<it->first<<" "<<it->second<<std::endl;
  }*/
}

void Acceleration::tearDown()
{
}


void Acceleration::receiveCombinedMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea> currentFrame, cluon::data::TimeStamp sampleTime){
  std::reverse_iterator<std::map<int,opendlv::logic::perception::GroundSurfaceArea>::iterator> it;
  it = currentFrame.rbegin();
  int I = 0;
  Eigen::MatrixXf localPath(currentFrame.size()*2,2);
  while(it != currentFrame.rend()){
    auto surfaceArea = it->second;
    float x1 = surfaceArea.x1(); //Unpack message
    float y1 = surfaceArea.y1();
    float x2 = surfaceArea.x2();
    float y2 = surfaceArea.y2();
    float x3 = surfaceArea.x3();
    float y3 = surfaceArea.y3();
    float x4 = surfaceArea.x4();
    float y4 = surfaceArea.y4();

    localPath(2*I,0)=(x1+x2)/2.0f;
    localPath(2*I,1)=(y1+y2)/2.0f;
    localPath(2*I+1,0)=(x3+x4)/2.0f;
    localPath(2*I+1,1)=(y3+y4)/2.0f;
    it++;
    I++;
  }
  Acceleration::run(localPath, sampleTime);
} // End of recieveCombinedMessage


void Acceleration::nextContainer(cluon::data::Envelope &a_container)
{
  /*if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    auto groundSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(a_container));
    m_groundSpeed = groundSpeed.groundSpeed();
  }*/
  if (a_container.dataType() == opendlv::proxy::GroundSpeedReading::ID()) {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    auto vehicleSpeed = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(std::move(a_container));
    if (m_speedId2>0) {
      if(a_container.senderStamp()==m_speedId1){
        m_groundSpeedReadingLeft = vehicleSpeed.groundSpeed();
      } else if (a_container.senderStamp()==m_speedId2){
        m_groundSpeedReadingRight = vehicleSpeed.groundSpeed();
      }
      m_groundSpeed = (m_groundSpeedReadingLeft + m_groundSpeedReadingRight)*0.5f;
    }
    else {
      m_groundSpeed = vehicleSpeed.groundSpeed();
    }
  }
  else if (a_container.dataType() == opendlv::logic::perception::GroundSurfaceProperty::ID()) {
    m_STOP = true;
  }
}

void Acceleration::run(Eigen::MatrixXf localPath, cluon::data::TimeStamp sampleTime){
  bool noPath = false;
  int count = 0;
  float headingRequest;
  float distanceToAimPoint;
  float accelerationRequest;
  float groundSpeedCopy;
  {
    std::unique_lock<std::mutex> lockGroundSpeed(m_groundSpeedMutex);
    groundSpeedCopy = m_groundSpeed;
  }

  if (localPath.rows()>2) {
    if(!m_usePathMemory){
      // Order path
      localPath = orderCones(localPath);
      // Remove negative path points
      if (localPath(0,0)<0.0f) {
        while (localPath(count,0)<0.0f){
          count++;
          if (count>localPath.rows()-1) {
            noPath = true;
            break;
          }
        }
      }
    }
    if (!noPath) {
      if (count>0) {
        Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
        localPath.resize(localPath.rows()-count,2);
        localPath = localPathTmp;
      }
      if (localPath.rows()>=2) {
        auto steering = Acceleration::driverModelSteering(localPath, groundSpeedCopy);
        headingRequest = std::get<0>(steering);
        distanceToAimPoint = std::get<1>(steering);
      }
    }
  }
  else {
    noPath=true;
  }

  if (noPath) {
    headingRequest=m_prevHeadingRequest;
    distanceToAimPoint=1.0f;
  }
  accelerationRequest = Acceleration::driverModelVelocity(groundSpeedCopy);

  { /*---SEND---*/
    std::unique_lock<std::mutex> lockSend(m_sendMutex);
    opendlv::logic::action::AimPoint steer;
    steer.azimuthAngle(headingRequest);
    steer.distance(distanceToAimPoint);
    m_od4.send(steer, sampleTime, m_senderStamp);

    if (accelerationRequest >= 0.0f) {
      opendlv::proxy::GroundAccelerationRequest acc;
      acc.groundAcceleration(accelerationRequest);
      m_od4.send(acc, sampleTime, m_senderStamp);
    }
    else if(accelerationRequest < 0.0f){
      opendlv::proxy::GroundDecelerationRequest dec;
      dec.groundDeceleration(-accelerationRequest);
      m_od4.send(dec, sampleTime, m_senderStamp);
    }
  } //end mutex scope
  if(m_STOP && groundSpeedCopy<0.1f){ //TODO: make sure it works
    opendlv::proxy::SwitchStateReading message;
    message.state(1);
    m_od4BB.send(message,sampleTime, 1403);
  }
}//end run


Eigen::MatrixXf Acceleration::orderCones(Eigen::MatrixXf localPath)
{
  // (A function from DetectConeLane)

  // Input: Cone and vehicle positions in the same coordinate system
  // Output: The localPath in order
  int nCones = localPath.rows();
  Eigen::MatrixXf current(1,2);
  current << -3,0;
  Eigen::ArrayXXi found(nCones,1);
  found.fill(-1);
  Eigen::MatrixXf orderedCones(nCones,2);
  float shortestDist;
  float tmpDist;
  int closestConeIndex = 0;

  // The first chosen cone is the one closest to the vehicle. After that it continues with the closest neighbour
  for(int i = 0; i < nCones; i = i+1)
  {

    shortestDist = std::numeric_limits<float>::infinity();
    // Find closest cone to the last chosen cone
    for(int j = 0; j < nCones; j = j+1)
    {
      if(!((found==j).any()))
      {
        tmpDist = (current-localPath.row(j)).norm();
        if(tmpDist < shortestDist)
        {
          shortestDist = tmpDist;
          closestConeIndex = j;
        } // End of if
      } // End of if
    } // End of for

    found(i) = closestConeIndex;
    current = localPath.row(closestConeIndex);
  } // End of for
  // Rearrange localPath to have the order of found
  for(int i = 0; i < nCones; i = i+1)
  {
    orderedCones.row(i) = localPath.row(found(i));
  } // End of for
  return orderedCones;
} // End of orderCones


void Acceleration::Cartesian2Spherical(float x, float y, float z, opendlv::logic::sensation::Point &pointInSpherical)
{
  double distance = sqrt(x*x+y*y+z*z);
  double azimuthAngle = atan2(y,x);
  double zenithAngle = atan2(z,sqrt(x*x+y*y));
  pointInSpherical.distance(static_cast<float>(distance));
  pointInSpherical.azimuthAngle(static_cast<float>(azimuthAngle));
  pointInSpherical.zenithAngle(static_cast<float>(zenithAngle));
} // End of Cartesian2Spherical


std::tuple<float, float> Acceleration::driverModelSteering(Eigen::MatrixXf localPath, float groundSpeedCopy) {
  float headingRequest = 0.0f;
  float distanceToAimPoint = 0.0f;
  bool noPath = false;
  if (m_moveOrigin) {
    // Move localPath to front wheel axis
    Eigen::MatrixXf foo = Eigen::MatrixXf::Zero(localPath.rows(),2);
    foo.col(0).fill(m_frontToCog);
    localPath = localPath-foo;

    if(!m_usePathMemory){
      // Remove negative path points
      if (localPath(0,0)<0.0f && localPath.rows()>0) {
        int count = 0;
        while (localPath(count,0)<0.0f){
          count++;
          if (count>localPath.rows()-1) {
            noPath = true;
            break;
          }
        }
        if(!noPath && count>0){
          Eigen::MatrixXf localPathTmp = localPath.bottomRows(localPath.rows()-count);
          localPath.resize(localPath.rows()-count,2);
          localPath = localPathTmp;
        }
      }
    }

  }

  if(!m_latestPathSet){
    m_latestPath = localPath;
    m_latestPathSet = true;
  }
  m_steerTockDt = std::chrono::system_clock::now();
  std::chrono::duration<float> DT = m_steerTockDt-m_steerTickDt;
  m_steerTickDt = std::chrono::system_clock::now();
  float dt = (DT.count()<1.0f) ? (DT.count()) : (0.1f); // Avoid large DT's to give high control outputs
  m_distanceTraveled+=groundSpeedCopy*dt;

  if(m_usePathMemory && abs(localPath(0,0)-localPath(1,0)) < 0.00001f && abs(localPath(0,1)-localPath(1,1)) < 0.00001f){
    // The two first points are the same, this means that the path has been updated.
    // Decrease the aim distance equally as much as the path has moved.
    m_aimDistance = m_aimDistance - m_distanceTraveled; //(localPath.row(0)-m_latestPath.row(0)).norm();
    m_distanceTraveled=0.0f;
  }

  Eigen::MatrixXf vectorFromPath = localPath.row(localPath.rows()-1)-localPath.row(0);
  vectorFromPath = vectorFromPath/(vectorFromPath.norm());
  Eigen::MatrixXf aimp1 = localPath.row(0) + m_aimDistance*vectorFromPath;

  if(m_usePathMemory && aimp1.norm() < 10){
    if(m_useAimDistanceLapCounter){
      m_STOP = true;
    }
    m_aimDistance = m_aimDistance + 50.0f;
    aimp1 = localPath.row(0) + m_aimDistance*vectorFromPath;
  }

  float pathPointAimDistance;
  if(!m_useDynamicTrust && m_staticTrustInLastPathPoint > 0.99f){
    // For easier visualization in this simple case
    pathPointAimDistance = (localPath.row(localPath.rows()-1)).norm();
  }else{
    pathPointAimDistance = m_aimDistance;
  }

  Eigen::MatrixXf vectorFromVehicle = localPath.row(localPath.rows()-1)/((localPath.row(localPath.rows()-1)).norm());
  Eigen::MatrixXf aimp2 = pathPointAimDistance*vectorFromVehicle;

  m_latestPath.resize(localPath.rows(),localPath.cols());
  m_latestPath = localPath;

  float trustInLastPathPoint;
  if(m_useDynamicTrust){
    // If distance is between two limits, trust increases linearly
    float distanceToLastPathPoint = (localPath.row(localPath.rows()-1)).norm();
    if(distanceToLastPathPoint < m_lowTrustLimDistance){
      trustInLastPathPoint = m_lowTrustLim;
    }else if(distanceToLastPathPoint > m_highTrustLimDistance){
      trustInLastPathPoint = m_highTrustLim;
    }else{
      trustInLastPathPoint = (distanceToLastPathPoint - m_lowTrustLimDistance)*(m_highTrustLim - m_lowTrustLim)/(m_highTrustLimDistance - m_lowTrustLimDistance) + m_lowTrustLim;
    } // End of else
  }else{
    if(m_usePathMemory){
      trustInLastPathPoint = 0;
    }else{
      trustInLastPathPoint = m_staticTrustInLastPathPoint;
    }
  } // End of else

  Eigen::MatrixXf combinedAimPoint = aimp1 + trustInLastPathPoint*(aimp2 - aimp1);
  opendlv::logic::sensation::Point sphericalPoint;
  Cartesian2Spherical(combinedAimPoint(0,0), combinedAimPoint(0,1), 0, sphericalPoint);
  headingRequest = sphericalPoint.azimuthAngle();
  distanceToAimPoint = sphericalPoint.distance();

  // Limit heading request due to physical limitations
  if (headingRequest>=0) {
    headingRequest = std::min(headingRequest,m_wheelAngleLimit*m_PI/180.0f);
  } else {
    headingRequest = std::max(headingRequest,-m_wheelAngleLimit*m_PI/180.0f);
  }
  if (m_prevReqRatio > 0.0f) {
    headingRequest = m_prevReqRatio*m_prevHeadingRequest + (1.0f-m_prevReqRatio)*headingRequest;
  }

  if (std::abs(headingRequest-m_prevHeadingRequest)/dt>(m_steerRate*m_PI/180.0)){
    if (headingRequest > m_prevHeadingRequest) {
      headingRequest = dt*m_steerRate*m_PI/180.0f + m_prevHeadingRequest;
    }
    else{
      headingRequest = -dt*m_steerRate*m_PI/180.0f + m_prevHeadingRequest;
    }
  }
  m_prevHeadingRequest=headingRequest;


  return std::make_tuple(headingRequest,distanceToAimPoint);
}


float Acceleration::driverModelVelocity(float groundSpeedCopy){
  float accelerationRequest=0.0f;
  m_tockDt = std::chrono::system_clock::now();
  std::chrono::duration<float> DT = m_tockDt-m_tickDt;
  float dt = (DT.count()<1.0f) ? (DT.count()) : (0.1f); // Avoid large DT's to give high control outputs
  float e = 0.0f;
  float ed = 0.0f;
  if (m_STOP && m_accelerate) {
    m_changeState = true;
    m_accelerate = false;
  }
  if (m_changeState) {
    m_ei=0.0f;
    m_ePrev=0.0f;
    m_changeState=false;
    //std::cout<<"CHANGE STATE"<<std::endl;
  }
  if (m_accelerate){
    e = m_velocityLimit-groundSpeedCopy;
    if (groundSpeedCopy<=0.0f) {
      m_ei=0.0f;
    }
    else{
      m_ei += e*dt;
    }
    m_ei=std::min(m_ei,m_axLimitPositive/m_aKi);
    ed = (e-m_ePrev)/dt;
    m_ePrev=e;
    float accTmp = m_aKp*e+m_aKd*ed+m_aKi*m_ei;
    accelerationRequest = std::min(std::abs(accTmp),m_axLimitPositive);
    if (accTmp<0) {
      accelerationRequest=-accelerationRequest;
    }
  }
  else if (m_STOP) {
    e = 0.0f-groundSpeedCopy;
    m_ei += e*dt;
    m_ei=std::max(m_ei,m_axLimitNegative/m_bKi);
    ed = (e-m_ePrev)/dt;
    m_ePrev=e;
    float accTmp = m_bKp*e+m_bKd*ed+m_bKi*m_ei;
    accelerationRequest = std::max(accTmp,m_axLimitNegative);
    if (accelerationRequest>0.0f) {
      accelerationRequest=0.0f;
    }
  }
  m_tickDt = std::chrono::system_clock::now();
  return accelerationRequest;
}
