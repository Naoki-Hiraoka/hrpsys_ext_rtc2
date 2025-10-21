#include "EmergencyStopper2.h"

#include <rtm/CorbaNaming.h>

#include <cnoid/BodyLoader>
#include <vector>
#include <limits>

static const char* emergencystopper_spec[] =
  {
    "implementation_id", "EmergencyStopper2",
    "type_name",         "EmergencyStopper2",
    "description",       "emergency stopper",
    "version",           "0.0.0",
    "vendor",            "Naoki-Hiraoka",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    "conf.default.debugLevel", "0",
    ""
  };

EmergencyStopper2::EmergencyStopper2(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
  m_qRefIn("qRef", m_qRef),
  m_tauRefIn("tauRef", m_tauRef),
  m_qActIn("qAct", m_qAct),
  m_tauCtlIn("tauCtl", m_tauCtl),
  m_stopSignalIn("stopSignal", m_stopSignal),
  m_releaseSignalIn("releaseSignal", m_releaseSignal),
  m_qOut("q", m_q),
  m_tauOut("tau", m_tau),
  m_EmergencyStopper2ServicePort("EmergencyStopper2Service"),
  m_RobotHardwareServicePort("RobotHardwareService")
{
  this->m_service0.setComp(this);
}

RTC::ReturnCode_t EmergencyStopper2::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  addInPort("qRef", m_qRefIn);
  addInPort("tauRef", m_tauRefIn);
  addInPort("qAct", m_qActIn);
  addInPort("tauCtl", m_tauCtlIn);
  addInPort("stopSignal", m_stopSignalIn);
  addInPort("releaseSignal", m_releaseSignalIn);
  addOutPort("q", m_qOut);
  addOutPort("tau", m_tauOut);
  m_EmergencyStopper2ServicePort.registerProvider("service0", "EmergencyStopper2Service", m_service0);
  addPort(m_EmergencyStopper2ServicePort);
  m_RobotHardwareServicePort.registerConsumer("service0", "RobotHardwareService", m_robotHardwareService0);
  addPort(m_RobotHardwareServicePort);

  RTC::Properties& prop = this->getProperties();

  cnoid::BodyLoader bodyLoader;
  std::string fileName;
  if(prop.hasKey("model")) fileName = std::string(prop["model"]);
  else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
  if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
  std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
  this->robot = bodyLoader.load(fileName);
  if(!this->robot){
    std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }

  this->joints.resize(this->robot->numJoints());
  return RTC::RTC_OK;
}

RTC::ReturnCode_t EmergencyStopper2::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << this->m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  for(int i=0;i<this->joints.size();i++) this->joints[i].init();
  return RTC::RTC_OK;
}

RTC::ReturnCode_t EmergencyStopper2::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << this->m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t EmergencyStopper2::onExecute(RTC::UniqueId ec_id)
{
  std::lock_guard<std::mutex> guard(this->mutex_);

  double rate = this->get_context(ec_id)->get_rate();
  if(rate <= 0.0){
    std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " periodic rate is invalid " << rate << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }
  double dt = 1.0 / rate;

  if (this->m_qRefIn.isNew()) {
    while (this->m_qRefIn.isNew()) this->m_qRefIn.read();
    if(this->m_qRef.data.length() == this->robot->numJoints()){
      for(int i=0;i<this->robot->numJoints();i++) this->joints[i].qRef = this->m_qRef.data[i];
    }else{
      return RTC::RTC_OK;  // qRef が届かなければ何もしない
    }
  }else{
    return RTC::RTC_OK;  // qRef が届かなければ何もしない
  }
  if (this->m_tauRefIn.isNew()) {
    while (this->m_tauRefIn.isNew()) this->m_tauRefIn.read();
    if(this->m_tauRef.data.length() == this->robot->numJoints()){
      for(int i=0;i<this->robot->numJoints();i++) this->joints[i].tauRef = this->m_tauRef.data[i];
    }
  }
  if (this->m_qActIn.isNew()) {
    while (this->m_qActIn.isNew()) this->m_qActIn.read();
    if(this->m_qAct.data.length() == this->robot->numJoints()){
      for(int i=0;i<this->robot->numJoints();i++) this->joints[i].qAct = this->m_qAct.data[i];
    }
  }
  if (this->m_tauCtlIn.isNew()) {
    while (this->m_tauCtlIn.isNew()) this->m_tauCtlIn.read();
    if(this->m_tauCtl.data.length() == this->robot->numJoints()){
      for(int i=0;i<this->robot->numJoints();i++) this->joints[i].tauCtl = this->m_tauCtl.data[i];
    }
  }
  if (this->m_stopSignalIn.isNew()) {
    while (this->m_stopSignalIn.isNew()) this->m_stopSignalIn.read();
    if(this->m_stopSignal.data) {
      for (int i=0; i<this->robot->numJoints(); i++) this->joints[i].stopMotion();
    }
  }
  if (this->m_releaseSignalIn.isNew()) {
    while (this->m_releaseSignalIn.isNew()) this->m_releaseSignalIn.read();
    if(this->m_releaseSignal.data) {
      for (int i=0; i<this->robot->numJoints(); i++) this->joints[i].releaseMotion(this->recover_time);
    }
  }

  for(int i=0;i<this->robot->numJoints();i++){
    if(this->joints[i].mode == Joint::MODE_REF){
      this->joints[i].qOut.setGoal(this->joints[i].qRef, this->joints[i].interpolationTime);
      this->joints[i].tauOut = this->joints[i].tauRef;
      if(this->joints[i].changeGain){
        if(!CORBA::is_nil(this->m_robotHardwareService0._ptr()) && //コンシューマにプロバイダのオブジェクト参照がセットされていない(接続されていない)状態
           !this->m_robotHardwareService0->_non_existent()){ //プロバイダのオブジェクト参照は割り当てられているが、相手のオブジェクトが非活性化 (RTC は Inactive 状態) になっている状態
          this->m_robotHardwareService0->setServoPGainPercentageWithTime(this->robot->joint(i)->name().c_str(),100.0,this->servoon_time);
          this->m_robotHardwareService0->setServoDGainPercentageWithTime(this->robot->joint(i)->name().c_str(),100.0,this->servoon_time);
        }
        this->joints[i].changeGain = false;
      }
    }else if(this->joints[i].mode == Joint::MODE_STOP){
      this->joints[i].qOut.reset(this->joints[i].qOut.value());
      this->joints[i].tauOut = 0.0;
      if(this->joints[i].changeGain){
        if(!CORBA::is_nil(this->m_robotHardwareService0._ptr()) && //コンシューマにプロバイダのオブジェクト参照がセットされていない(接続されていない)状態
           !this->m_robotHardwareService0->_non_existent()){ //プロバイダのオブジェクト参照は割り当てられているが、相手のオブジェクトが非活性化 (RTC は Inactive 状態) になっている状態
          this->m_robotHardwareService0->setServoPGainPercentageWithTime(this->robot->joint(i)->name().c_str(),100.0,this->servoon_time);
          this->m_robotHardwareService0->setServoDGainPercentageWithTime(this->robot->joint(i)->name().c_str(),100.0,this->servoon_time);
        }
        this->joints[i].changeGain = false;
      }
    }else if(this->joints[i].mode == Joint::MODE_TORQUE){
      this->joints[i].qOut.setGoal(this->joints[i].qAct, this->joints[i].interpolationTime);
      this->joints[i].tauOut = this->joints[i].tauCtl;
      if(this->joints[i].changeGain){
        if(!CORBA::is_nil(this->m_robotHardwareService0._ptr()) && //コンシューマにプロバイダのオブジェクト参照がセットされていない(接続されていない)状態
           !this->m_robotHardwareService0->_non_existent()){ //プロバイダのオブジェクト参照は割り当てられているが、相手のオブジェクトが非活性化 (RTC は Inactive 状態) になっている状態
          this->m_robotHardwareService0->setServoPGainPercentageWithTime(this->robot->joint(i)->name().c_str(),0.0,this->servooff_time);
          this->m_robotHardwareService0->setServoDGainPercentageWithTime(this->robot->joint(i)->name().c_str(),0.0,this->servooff_time);
        }
        this->joints[i].changeGain = false;
      }
    }

    this->joints[i].qOut.interpolate(dt);
    this->joints[i].interpolationTime = std::max(this->joints[i].interpolationTime - dt, 0.0);
  }

  this->m_q.tm = this->m_qRef.tm;
  this->m_q.data.length(this->robot->numJoints());
  for(int i=0;i<this->robot->numJoints();i++) this->m_q.data[i] = this->joints[i].qOut.value();
  this->m_qOut.write();

  this->m_tau.tm = this->m_tauRef.tm;
  this->m_tau.data.length(this->robot->numJoints());
  for(int i=0;i<this->robot->numJoints();i++) this->m_tau.data[i] = this->joints[i].tauOut;
  this->m_tauOut.write();

  return RTC::RTC_OK;
}

bool EmergencyStopper2::stopMotion(const char *jname) {
  std::lock_guard<std::mutex> guard(this->mutex_);

  cnoid::LinkPtr l;
  if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
    for (int i=0; i<this->robot->numJoints(); i++) this->joints[i].stopMotion();
    std::cerr << "[" << this->m_profile.instance_name << "] stopMotion for all joints" << std::endl;
  }else if ((l = robot->link(jname)) && (l->jointId() >= 0)){
    this->joints[l->jointId()].stopMotion();
    std::cerr << "[" << this->m_profile.instance_name << "] stopMotion for " << jname << std::endl;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Invalid joint name of stopMotion " << jname << "!" << std::endl;
    return false;
  }
  return true;
}

void EmergencyStopper2::Joint::stopMotion(){
  if(this->mode != MODE_STOP){
    if(this->mode == MODE_TORQUE) this->changeGain = true;
    this->mode = MODE_STOP;
    this->interpolationTime = 0.0;
    this->qOut.reset(this->qOut.value());
  }
}

bool EmergencyStopper2::releaseMotion(const char *jname) {
  std::lock_guard<std::mutex> guard(this->mutex_);

  cnoid::LinkPtr l;
  if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
    for (int i=0; i<this->robot->numJoints(); i++) this->joints[i].releaseMotion(this->recover_time);
    std::cerr << "[" << this->m_profile.instance_name << "] releaseMotion for all joints" << std::endl;
  }else if ((l = robot->link(jname)) && (l->jointId() >= 0)){
    this->joints[l->jointId()].releaseMotion(this->recover_time);
    std::cerr << "[" << this->m_profile.instance_name << "] releaseMotion for " << jname << std::endl;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Invalid joint name of releaseMotion " << jname << "!" << std::endl;
    return false;
  }
  return true;
}

void EmergencyStopper2::Joint::releaseMotion(double recover_time){
  if(this->mode != MODE_REF){
    if(this->mode == MODE_TORQUE) this->changeGain = true;
    this->mode = MODE_REF;
    this->interpolationTime = recover_time;
    this->qOut.reset(this->qOut.value());
  }
}

bool EmergencyStopper2::startTorque(const char *jname) {
  std::lock_guard<std::mutex> guard(this->mutex_);

  cnoid::LinkPtr l;
  if (strcmp(jname, "all") == 0 || strcmp(jname, "ALL") == 0){
    for (int i=0; i<this->robot->numJoints(); i++) this->joints[i].startTorque(this->retrieve_time);
    std::cerr << "[" << this->m_profile.instance_name << "] startTorque for all joints" << std::endl;
  }else if ((l = robot->link(jname)) && (l->jointId() >= 0)){
    this->joints[l->jointId()].startTorque(this->retrieve_time);
    std::cerr << "[" << this->m_profile.instance_name << "] startTorque for " << jname << std::endl;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] Invalid joint name of startTorque " << jname << "!" << std::endl;
    return false;
  }
  return true;
}

void EmergencyStopper2::Joint::startTorque(double retrieve_time){
  if(this->mode != MODE_TORQUE){
    this->mode = MODE_TORQUE;
    this->interpolationTime = retrieve_time;
    this->changeGain = true;
    this->qOut.reset(this->qOut.value());
  }
}

bool EmergencyStopper2::setEmergencyStopper2Param(const hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  this->recover_time = i_param.recover_time;
  this->retrieve_time = i_param.retrieve_time;
  return true;
}

bool EmergencyStopper2::getEmergencyStopper2Param(hrpsys_ext_rtc::EmergencyStopper2Service::EmergencyStopper2Param& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  i_param.recover_time = this->recover_time;
  i_param.retrieve_time = this->retrieve_time;
  return true;
}

extern "C"
{

  void EmergencyStopper2Init(RTC::Manager* manager)
  {
    RTC::Properties profile(emergencystopper_spec);
    manager->registerFactory(profile,
                             RTC::Create<EmergencyStopper2>,
                             RTC::Delete<EmergencyStopper2>);
  }

};


