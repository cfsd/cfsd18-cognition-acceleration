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

#ifndef CFSD18_COGNITION_ACCELERATION_HPP
#define CFSD18_COGNITION_ACCELERATION_HPP

#include <opendlv-standard-message-set.hpp>
#include <cluon-complete.hpp>
#include <Eigen/Dense>
#include <map>
#include <chrono>
#include <thread>

class Acceleration {
 public:
  Acceleration(std::map<std::string, std::string> commandlineArguments, cluon::OD4Session &od4);
  Acceleration(Acceleration const &) = delete;
  Acceleration &operator=(Acceleration const &) = delete;
  virtual ~Acceleration();
  void receiveCombinedMessage(std::map<int,opendlv::logic::perception::GroundSurfaceArea>, cluon::data::TimeStamp);
  void nextContainer(cluon::data::Envelope &);

 private:
  void setUp(std::map<std::string, std::string> commandlineArguments);
  void tearDown();

  void run(Eigen::MatrixXf localPath, cluon::data::TimeStamp sampleTime);
  Eigen::MatrixXf orderCones(Eigen::MatrixXf localPath);
  void Cartesian2Spherical(float, float, float, opendlv::logic::sensation::Point &);
  std::tuple<float, float> driverModelSteering(Eigen::MatrixXf);
  float driverModelVelocity(float, float);

  /* commandlineArguments */
  cluon::OD4Session &m_od4;
  int m_senderStamp{221};
  // steering
  bool m_usePathMemory{true};
  float m_staticTrustInLastPathPoint{0.5f};
  bool m_useDynamicTrust{false};
  float m_lowTrustLimDistance{10.0f};
  float m_highTrustLimDistance{20.0f};
  float m_lowTrustLim{0.0f};
  float m_highTrustLim{1.0f};
  float m_aimDistance{50.0f};
  bool m_moveOrigin{true};
  float m_steerRate{50.0f};
  // velocity control
  bool m_useAyReading{false};
  float m_velocityLimit{5.0f};
  float m_mu{0.9f};
  float m_axLimitPositive{5.0f};
  float m_axLimitNegative{-5.0f};
  float m_headingErrorDependency{0.7f};
  //....controller
  float m_aimVel{5.0f};
  float m_aKp{0.1f};
  float m_aKd{0.0f};
  float m_aKi{0.0f};
  float m_bKp{0.1f};
  float m_bKd{0.0f};
  float m_bKi{0.0f};
  float m_sKp{0.1f};
  float m_sKd{0.0f};
  float m_sKi{0.0f};
  // vehicle specific
  float m_wheelAngleLimit{20.0f};
  float m_wheelBase{1.53f};
  float m_frontToCog{0.765f};

  /* Member variables */
  float const m_PI = 3.14159265f;
  float m_groundSpeed;
  std::mutex m_groundSpeedMutex;
  float m_lateralAcceleration;
  std::mutex m_lateralAccelerationMutex;
  std::chrono::time_point<std::chrono::system_clock> m_tickDt;
  std::chrono::time_point<std::chrono::system_clock> m_tockDt;
  std::chrono::time_point<std::chrono::system_clock> m_steerTickDt;
  std::chrono::time_point<std::chrono::system_clock> m_steerTockDt;
  float m_ei;
  float m_ePrev;
  bool m_changeState;
  float m_prevHeadingRequest;
  bool m_accelerate;
  bool m_start;
  bool m_STOP;
  std::mutex m_sendMutex;
};

#endif
