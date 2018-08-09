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
  std::tuple<float, float> driverModelSteering(Eigen::MatrixXf, float);
  float driverModelVelocity(float);
  float lowPass(int factor, float lastOutput, float presentReading);

  /* commandlineArguments */
  cluon::OD4Session &m_od4;
  cluon::OD4Session m_od4BB{219};
  uint32_t m_speedId1{1504};
  uint32_t m_speedId2{0};
  int m_senderStamp{221};
  // steering
  float m_correctionCooldown{0.3f};
  float m_aimDeltaLimit{3.0f};
  float m_k{2.0f};
  bool m_useSteerRateControl{};
  bool m_usePathMemory{};
  bool m_useAimDistanceLapCounter{};
  float m_staticTrustInLastPathPoint{0.5f};
  bool m_useDynamicTrust{false};
  float m_lowTrustLimDistance{10.0f};
  float m_highTrustLimDistance{20.0f};
  float m_lowTrustLim{0.0f};
  float m_highTrustLim{1.0f};
  float m_aimDistance{50.0f};
  bool m_moveOrigin{true};
  float m_steerRate{50.0f};
  float m_prevReqRatio{0.0f};
  float m_aimRate{100.0f};
  float m_aimFreq{1000.0f};
  float m_prevAimReqRatio{0.0f};
  bool m_useYawRate{true};
  int m_lowPassfactor{0};
  // velocity control
  float m_velocityLimit{20.0f};
  float m_axLimitPositive{5.0f};
  float m_axLimitNegative{-5.0f};
  //....controller
  float m_aKp{1.0f};
  float m_aKd{0.0f};
  float m_aKi{0.1f};
  float m_bKp{1.0f};
  float m_bKd{0.0f};
  float m_bKi{0.1f};
  float m_sKp{1.0f};
  float m_sKd{0.2f};
  float m_sKi{0.0f};
  // vehicle specific
  float m_wheelAngleLimit{30.0f};
  float m_frontToCog{0.765f};

  /* Member variables */
  float const m_PI = 3.14159265f;
  float m_groundSpeed;
  std::mutex m_groundSpeedMutex;
  std::chrono::time_point<std::chrono::system_clock> m_tickDt;
  std::chrono::time_point<std::chrono::system_clock> m_tockDt;
  std::chrono::time_point<std::chrono::system_clock> m_steerTickDt;
  std::chrono::time_point<std::chrono::system_clock> m_steerTockDt;
  float m_ei;
  float m_ePrev;
  bool m_changeState;
  float m_prevHeadingRequest;
  bool m_accelerate;
  bool m_STOP;
  bool m_latestPathSet;
  Eigen::MatrixXf m_latestPath;
  float m_distanceTraveled;
  float m_groundSpeedReadingLeft;
  float m_groundSpeedReadingRight;
  std::mutex m_yawMutex;
  float m_yawRate;
  float m_sEPrev;
  float m_sEi;
  float m_aimClock;
  float m_prevAngleToAimPoint;
  float m_aimPointRate;
  int m_rateCount;
  float m_timeSinceLastCorrection;

  std::mutex m_sendMutex;
};

#endif
