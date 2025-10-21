// -*- C++ -*-
/*!
 * @file  SoftErrorLimiter2.cpp
 * @brief soft error limiter
 * $Date$
 *
 * $Id$
 */

#include "SoftErrorLimiter2.h"
#include "hrpsys/util/VectorConvert.h"
#include "hrpsys/idl/RobotHardwareService.hh"
#include <rtm/CorbaNaming.h>

#include <cnoid/BodyLoader>
#include <math.h>
#include <vector>
#include <limits>
#include <iomanip>
#define deg2rad(x)((x)*M_PI/180)

// Module specification
// <rtc-template block="module_spec">
static const char* softerrorlimiter_spec[] =
  {
    "implementation_id", "SoftErrorLimiter2",
    "type_name",         "SoftErrorLimiter2",
    "description",       "soft error limiter",
    "version",           "0.0.0",
    "vendor",            "Naoki-Hiraoka",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

SoftErrorLimiter2::SoftErrorLimiter2(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qRefIn("qRef", m_qRef),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_servoStateIn("servoStateIn", m_servoState),
    m_qOut("q", m_qRef),
    m_servoStateOut("servoStateOut", m_servoState),
    m_beepCommandOut("beepCommand", m_beepCommand),
    m_SoftErrorLimiter2ServicePort("SoftErrorLimiter2Service"),
    // </rtc-template>
    m_debugLevel(0),
    dummy(0),
    is_beep_port_connected(false)
{
  init_beep();
  start_beep(3136);
}

SoftErrorLimiter2::~SoftErrorLimiter2()
{
  quit_beep();
}



RTC::ReturnCode_t SoftErrorLimiter2::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("servoState", m_servoStateIn);
  
  // Set OutPort buffer
  addOutPort("q", m_qOut);
  addOutPort("servoState", m_servoStateOut);
  addOutPort("beepCommand", m_beepCommandOut);
  
  // Set service provider to Ports
  m_SoftErrorLimiter2ServicePort.registerProvider("service0", "SoftErrorLimiter2Service", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_SoftErrorLimiter2ServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  m_robot = bodyLoader.load(fileName);
  if(!m_robot){
    std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  std::cout << "[" << m_profile.instance_name << "] dof = " << m_robot->numJoints() << std::endl;
  m_service0.setComp(this);
  m_servoState.data.length(m_robot->numJoints());
  for(unsigned int i = 0; i < m_robot->numJoints(); i++) {
    m_servoState.data[i].length(1);
    int status = 0;
    status |= 1<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
    status |= 1<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
    status |= 1<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
    status |= 0<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
    status |= 0<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
    m_servoState.data[i][0] = status;
  }

  m_beepCommand.data.length(bc.get_num_beep_info());

  // load joint limit table
  std::string jointLimitTableProp;
  if(this->getProperties().hasKey("joint_limit_table")) jointLimitTableProp = std::string(this->getProperties()["joint_limit_table"]);
  else jointLimitTableProp = std::string(this->m_pManager->getConfig()["joint_limit_table"]); // 引数 -o で与えたプロパティを捕捉
  std::cerr << "[" << this->m_profile.instance_name << "] joint_limit_table: " << jointLimitTableProp<<std::endl;
  std::vector<std::shared_ptr<joint_limit_table::JointLimitTable> > jointLimitTablesVec
    = joint_limit_table::readJointLimitTablesFromProperty (m_robot, jointLimitTableProp);
  for(size_t i=0;i<jointLimitTablesVec.size();i++){
    joint_limit_tables[jointLimitTablesVec[i]->getSelfJoint()->name()] = jointLimitTablesVec[i];
  }

  // read ignored joint
  std::string maskJointLimitProp;
  if(this->getProperties().hasKey("mask_joint_limit")) maskJointLimitProp = std::string(this->getProperties()["mask_joint_limit"]);
  else maskJointLimitProp = std::string(this->m_pManager->getConfig()["mask_joint_limit"]); // 引数 -o で与えたプロパティを捕捉
  m_joint_mask.resize(m_robot->numJoints(), false);
  coil::vstring ijoints = coil::split(maskJointLimitProp.c_str(), ",");
  for(int i = 0; i < ijoints.size(); i++) {
      cnoid::LinkPtr lk = m_robot->link(std::string(ijoints[i]));
      if((!!lk) && (lk->jointId() >= 0)) {
          std::cout << "[" << m_profile.instance_name << "] "
                    << "disable ErrorLimit, joint : " << ijoints[i]
                    << " (id = " << lk->jointId() << ")" << std::endl;
          m_joint_mask[lk->jointId()] = true;
      }
  }

  m_servoErrorLimit.resize(m_robot->numJoints());
  for (unsigned int i=0; i<m_robot->numJoints(); i++){
    m_servoErrorLimit[i] = (0.2 - 0.02); // [rad]
  }

  m_positionLimitSatisfiedOnceBefore.resize(m_robot->numJoints(), false);
  m_servo_state.resize(m_robot->numJoints(), 0);

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SoftErrorLimiter2::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter2::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter2::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SoftErrorLimiter2::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftErrorLimiter2::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftErrorLimiter2::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static int loop = 0;
  static bool debug_print_velocity_first = false;
  static bool debug_print_position_first = false;
  static bool debug_print_error_first = false;
  loop ++;

  double dt = 1.0 / this->get_context(ec_id)->get_rate();

  /* Calculate count for beep sound frequency */
  int soft_limit_error_beep_freq = static_cast<int>(1.0/(4.0*dt)); // soft limit error => 4 times / 1[s]
  int position_limit_error_beep_freq = static_cast<int>(1.0/(2.0*dt)); // position limit error => 2 times / 1[s]
  int debug_print_freq = static_cast<int>(0.2/dt); // once per 0.2 [s]

  // Connection check of m_beepCommand to BeeperRTC
  //   If m_beepCommand is not connected to BeeperRTC and sometimes, check connection.
  //   If once connection is found, never check connection.
  if (!is_beep_port_connected && (loop % 500 == 0) ) {
    if ( m_beepCommandOut.connectors().size() > 0 ) {
      is_beep_port_connected = true;
      quit_beep();
      std::cerr << "[" << m_profile.instance_name<< "] beepCommand data port connection found! Use BeeperRTC." << std::endl;
    }
  }

  if (m_qRefIn.isNew()) {
    while (m_qRefIn.isNew()) m_qRefIn.read();
  }
  if (m_qCurrentIn.isNew()) {
    while (m_qCurrentIn.isNew()) m_qCurrentIn.read();
  }
  if (m_servoStateIn.isNew()) {
    while (m_servoStateIn.isNew()) m_servoStateIn.read();
  }

  /*
    0x001 : 'SS_OVER_VOLTAGE',
    0x002 : 'SS_OVER_LOAD',
    0x004 : 'SS_OVER_VELOCITY',
    0x008 : 'SS_OVER_CURRENT',
    0x010 : 'SS_OVER_HEAT',
    0x020 : 'SS_TORQUE_LIMIT',
    0x040 : 'SS_VELOCITY_LIMIT',
    0x080 : 'SS_FORWARD_LIMIT',
    0x100 : 'SS_REVERSE_LIMIT',
    0x200 : 'SS_POSITION_ERROR',
    0x300 : 'SS_ENCODER_ERROR',
    0x800 : 'SS_OTHER'
  */
  bool soft_limit_error = false;
  bool velocity_limit_error = false;
  bool position_limit_error = false;
  if ( m_qRef.data.length() == m_qCurrent.data.length() &&
       m_qRef.data.length() == m_servoState.data.length() ) {
    // prev_angle is previous output
    static std::vector<double> prev_angle;
    if ( prev_angle.size() != m_qRef.data.length() ) { // initialize prev_angle
      prev_angle.resize(m_qRef.data.length(), 0);
      for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ){
        prev_angle[i] = m_qCurrent.data[i];
      }
    }
    for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ){
      if(((m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT) == 1){
        m_servo_state[i] = 10;
      }else{
        m_servo_state[i] = std::max(0, m_servo_state[i] - 1);
      }
    }

      /*
        From hrpModel/Body.h
        inline Link* joint(int id) const
           This function returns a link that has a given joint ID.
           If there is no link that has a given joint ID,
           the function returns a dummy link object whose ID is minus one.
           The maximum id can be obtained by numJoints().

         inline Link* link(int index) const
           This function returns the link of a given index in the whole link sequence.
           The order of the sequence corresponds to a link-tree traverse from the root link.
           The size of the sequence can be obtained by numLinks().

         So use m_robot->joint(i) for llimit/ulimit, lvlimit/ulimit
       */

    // Velocity limitation for reference joint angles
    for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ){
      if(m_joint_mask[i]) continue;
      // Output angle should satisfy following limits.
      // 1. Velocity Limit (highest priority)
      // 2. Error Limit
      // 3. Position Limit (lowest priority)
      // If these limits conflict with each other, the one with the higher priority takes precedence.

      // Position limitation for reference joint angles
      {
      double llimit = m_robot->joint(i)->q_lower();
      double ulimit = m_robot->joint(i)->q_upper();
      if (joint_limit_tables.find(m_robot->joint(i)->name()) != joint_limit_tables.end()) {
          std::map<std::string, std::shared_ptr<joint_limit_table::JointLimitTable> >::iterator it = joint_limit_tables.find(m_robot->joint(i)->name());
          llimit = std::max(llimit, it->second->getLlimit(m_qRef.data[it->second->getTargetJoint()->jointId()]));
          ulimit = std::min(ulimit, it->second->getUlimit(m_qRef.data[it->second->getTargetJoint()->jointId()]));
      }
      // check only the joints which satisfied position limits once before
      if ( m_servo_state[i] >= 1 && (llimit <= m_qRef.data[i]) && (m_qRef.data[i] <= ulimit) ) m_positionLimitSatisfiedOnceBefore[i] = true;
      else if ( m_servo_state[i] == 0 ) m_positionLimitSatisfiedOnceBefore[i] = false;

      // fixed joint have vlimit = ulimit
      bool servo_limit_state = (llimit < ulimit) && ((llimit > m_qRef.data[i]) || (ulimit < m_qRef.data[i]));
      if ( m_servo_state[i] >= 1 && m_positionLimitSatisfiedOnceBefore[i] && servo_limit_state ) {
        if (loop % debug_print_freq == 0 || debug_print_position_first) {
          std::cerr << "[" << m_profile.instance_name<< "] [" << m_qRef.tm
                    << "] position limit over " << m_robot->joint(i)->name() << "(" << i << "), qRef=" << m_qRef.data[i]
                    << ", llimit =" << llimit
                    << ", ulimit =" << ulimit
                    << ", servo_state = " <<  ( m_servo_state[i] >= 1 ? "ON" : "OFF")
                    << ", prev_angle = " << prev_angle[i];
        }
        // fix joint angles
        m_qRef.data[i] = std::min(ulimit, std::max(llimit, m_qRef.data[i]));
        if (loop % debug_print_freq == 0 || debug_print_position_first ) {
            std::cerr << ", q(limited) = " << m_qRef.data[i] << std::endl;
        }
        m_servoState.data[i][0] |= (0x200 << OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT);
        position_limit_error = true;
      }
      }

      // Servo error limitation between reference joint angles and actual joint angles
      {
      double limit = std::max(0.0, m_servoErrorLimit[i]); // limit should be greater than or equal to zero.
      double error = m_qRef.data[i] - m_qCurrent.data[i];
      if ( m_servo_state[i] >= 1 && fabs(error) > limit ) {
        if (loop % debug_print_freq == 0 || debug_print_error_first ) {
          std::cerr << "[" << m_profile.instance_name<< "] [" << m_qRef.tm
                    << "] error limit over " << m_robot->joint(i)->name() << "(" << i << "), qRef=" << m_qRef.data[i]
                    << ", qCurrent=" << m_qCurrent.data[i] << " "
                    << ", Error=" << error << " > " << limit << " (limit)"
                    << ", servo_state = " <<  ( 1 ? "ON" : "OFF");
        }
        m_qRef.data[i] = std::max(m_qCurrent.data[i] - limit, std::min(m_qCurrent.data[i] + limit, m_qRef.data[i]));
        if (loop % debug_print_freq == 0 || debug_print_error_first ) {
          std::cerr << ", q(limited) = " << m_qRef.data[i] << std::endl;
        }
        m_servoState.data[i][0] |= (0x040 << OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT);
        soft_limit_error = true;
      }
      }

      // Velocity limitation
      {
      double qvel = (m_qRef.data[i] - prev_angle[i]) / dt;
      double lvlimit = m_robot->joint(i)->dq_lower() + 0.000175; // 0.01 deg / sec
      double uvlimit = m_robot->joint(i)->dq_upper() - 0.000175;
      // fixed joint has ulimit = vlimit
      if ( m_servo_state[i] >= 1 && (lvlimit < uvlimit) && ((lvlimit > qvel) || (uvlimit < qvel)) ) {
        if (loop % debug_print_freq == 0 || debug_print_velocity_first ) {
          std::cerr << "[" << m_profile.instance_name<< "] [" << m_qRef.tm
                    << "] velocity limit over " << m_robot->joint(i)->name() << "(" << i << "), qvel=" << qvel
                    << ", lvlimit =" << lvlimit
                    << ", uvlimit =" << uvlimit
                    << ", servo_state = " <<  ( m_servo_state[i] >= 1 ? "ON" : "OFF");
        }
        // fix joint angle
        m_qRef.data[i] = std::min(prev_angle[i] + uvlimit * dt, std::max(prev_angle[i] + lvlimit * dt, m_qRef.data[i]));
        if (loop % debug_print_freq == 0 || debug_print_velocity_first ) {
            std::cerr << ", q(limited) = " << m_qRef.data[i] << std::endl;
        }
        velocity_limit_error = true;
      }
      }

      prev_angle[i] = m_qRef.data[i];
    }
    // display error info if no error found
    debug_print_velocity_first = !velocity_limit_error;
    debug_print_position_first = !position_limit_error;
    debug_print_error_first = !soft_limit_error;

    // Beep sound
    if ( soft_limit_error ) { // play beep
      if (is_beep_port_connected) {
        if ( loop % soft_limit_error_beep_freq == 0 ) bc.startBeep(3136, soft_limit_error_beep_freq*0.8);
        else bc.stopBeep();
      } else {
        if ( loop % soft_limit_error_beep_freq == 0 ) start_beep(3136, soft_limit_error_beep_freq*0.8);
      }
    }else if ( position_limit_error || velocity_limit_error ) { // play beep
      if (is_beep_port_connected) {
        if ( loop % position_limit_error_beep_freq == 0 ) bc.startBeep(3520, position_limit_error_beep_freq*0.8);
        else bc.stopBeep();
      } else {
        if ( loop % position_limit_error_beep_freq == 0 ) start_beep(3520, position_limit_error_beep_freq*0.8);
      }
    } else {
      if (is_beep_port_connected) {
        bc.stopBeep();
      } else {
        stop_beep();
      }
    }
    m_qOut.write();
    m_servoStateOut.write();
  } else {
    if (is_beep_port_connected) {
      bc.startBeep(3136);
    } else {
      start_beep(3136);
    }
    if ( loop % 100 == 1 ) {
        std::cerr << "SoftErrorLimiter2 is not working..." << std::endl;
        std::cerr << "         m_qRef " << m_qRef.data.length() << std::endl;
        std::cerr << "     m_qCurrent " << m_qCurrent.data.length() << std::endl;
        std::cerr << "   m_servoState " << m_servoState.data.length() << std::endl;
    }
  }
  if (is_beep_port_connected) {
    bc.setDataPort(m_beepCommand);
    if (bc.isWritable()) m_beepCommandOut.write();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SoftErrorLimiter2::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter2::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter2::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter2::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter2::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

bool SoftErrorLimiter2::setServoErrorLimit(const char *jname, double limit) {
  cnoid::LinkPtr l;
  if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
    for (unsigned int i=0; i<m_robot->numJoints(); i++){
      m_servoErrorLimit[i] = limit;
    }
    std::cerr << "[el] setServoErrorLimit " << limit << "[rad] for all joints" << std::endl;
  }else if ((l = m_robot->link(jname)) && (l->jointId() >= 0)){
    m_servoErrorLimit[l->jointId()] = limit;
    std::cerr << "[el] setServoErrorLimit " << limit << "[rad] for " << jname << std::endl;
  }else{
    std::cerr << "[el] Invalid joint name of setServoErrorLimit " << jname << "!" << std::endl;
    return false;
  }
  return true;
}

extern "C"
{

  void SoftErrorLimiter2Init(RTC::Manager* manager)
  {
    RTC::Properties profile(softerrorlimiter_spec);
    manager->registerFactory(profile,
                             RTC::Create<SoftErrorLimiter2>,
                             RTC::Delete<SoftErrorLimiter2>);
  }

};


