#include <iomanip>
#include <rtm/CorbaNaming.h>
#include <cnoid/BodyLoader>
#include <hrpsys/idl/RobotHardwareService.hh>
#include "CollisionDetector2.h"
#include <choreonoid_qhull/choreonoid_qhull.h>

static const char* component_spec[] =
{
    "implementation_id", "CollisionDetector2",
    "type_name",         "CollisionDetector2",
    "description",       "collisoin detector component",
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

CollisionDetector2::CollisionDetector2(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_qRefIn("qRef", m_qRef),
      m_qCurrentIn("qCurrent", m_qCurrent),
      m_servoStateIn("servoStateIn", m_servoState),
      m_stopSignalOut("stopSignal", m_stopSignal),
      m_releaseSignalOut("releaseSignal", m_releaseSignal),
      m_CollisionDetector2ServicePort("CollisionDetector2Service")
{
    m_service0.collision(this);
}



RTC::ReturnCode_t CollisionDetector2::onInitialize()
{
    std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;

    addInPort("qRef", m_qRefIn);
    addInPort("qCurrent", m_qCurrentIn);
    addInPort("servoStateIn", m_servoStateIn);
    addOutPort("stopSignal", m_stopSignalOut);
    addOutPort("releaseSignal", m_releaseSignalOut);

    m_CollisionDetector2ServicePort.registerProvider("service0", "CollisionDetector2Service", m_service0);
    addPort(m_CollisionDetector2ServicePort);

    // load robot
    cnoid::BodyLoader bodyLoader;
    std::string fileName;
    if(this->getProperties().hasKey("model")) fileName = std::string(this->getProperties()["model"]);
    else fileName = std::string(this->m_pManager->getConfig()["model"]); // 引数 -o で与えたプロパティを捕捉
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    std::cerr << "[" << this->m_profile.instance_name << "] model: " << fileName <<std::endl;
    this->m_robot = bodyLoader.load(fileName);
    if(!this->m_robot){
      std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }

    choreonoid_qhull::convertAllCollisionToConvexHull(this->m_robot);
    this->setupVClipModel(this->m_robot);

    std::string collisionPairStr;
    if ( this->getProperties().hasKey("collision_pair")) collisionPairStr = std::string(this->getProperties()["collision_pair"]);
    else if(this->m_pManager->getConfig().hasKey("collision_pair")) collisionPairStr = std::string(this->m_pManager->getConfig()["collision_pair"]); // 引数 -o で与えたプロパティを捕捉
    std::cerr << "[" << m_profile.instance_name << "] prop[collision_pair] ->" << collisionPairStr << std::endl;
    std::istringstream iss(collisionPairStr);
    std::string tmp;
    while (getline(iss, tmp, ' ')) {
      size_t pos = tmp.find_first_of(':');
      std::string name1 = tmp.substr(0, pos), name2 = tmp.substr(pos+1);
      if ( m_robot->link(name1)==NULL ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find robot link " << name1 << std::endl;
        std::cerr << "[" << m_profile.instance_name << "] please choose one of following :";
        for (unsigned int i=0; i < m_robot->numLinks(); i++) {
          std::cerr << " " << m_robot->link(i)->name();
        }
        std::cerr << std::endl;
        continue;
      }
      if ( m_robot->link(name2)==NULL ) {
        std::cerr << "[" << m_profile.instance_name << "] Could not find robot link " << name2 << std::endl;
        std::cerr << "[" << m_profile.instance_name << "] please choose one of following :";
        for (unsigned int i=0; i < m_robot->numLinks(); i++) {
          std::cerr << " " << m_robot->link(i)->name();
        }
        std::cerr << std::endl;
        continue;
      }
      std::cerr << "[" << m_profile.instance_name << "] check collisions between " << m_robot->link(name1)->name() << " and " <<  m_robot->link(name2)->name() << std::endl;
      m_pair[tmp] = std::make_shared<CollisionLinkPair>(m_robot->link(name1), m_robot->link(name2));
    }

    this->m_qRefv = cnoid::VectorXd::Zero(m_robot->numJoints());
    this->m_qCurrentv = cnoid::VectorXd::Zero(m_robot->numJoints());
    this->m_servoStatev.resize(m_robot->numJoints(), false);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector2::onActivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
    this->m_enable = true;
    this->m_collisionFreeOnce = false;
    this->prevCollision = false;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector2::onDeactivated(RTC::UniqueId ec_id)
{
    std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
    return RTC::RTC_OK;
}

RTC::ReturnCode_t CollisionDetector2::onExecute(RTC::UniqueId ec_id)
{
    loop++;

    double rate = this->get_context(ec_id)->get_rate();
    if(rate <= 0.0){
      std::cerr << "\x1b[31m[" << m_profile.instance_name << "] " << " periodic rate is invalid " << rate << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    double dt = 1.0 / rate;

    /*
      サーボオン状態の関節がないとき: releaseSignal
      サーボオン状態の関節があるとき:
        サーボオンしてから一度も今の指令値が自己干渉しない姿勢になっていない場合: releaseSignal
        else:
          今の指令値が自己干渉する姿勢 かつ 次の指令値が今以上に自己干渉が悪化する姿勢に遷移するものだった場合: stopSignal
          else : release Signal
     */

    bool is_qRef_updated = false;
    if (this->m_qRefIn.isNew() ) {
      while (this->m_qRefIn.isNew() ) this->m_qRefIn.read();
      if(this->m_qRef.data.length() == this->m_robot->numJoints()){
        for(int i=0;i<this->m_robot->numJoints();i++) this->m_qRefv[i] = this->m_qRef.data[i];
        is_qRef_updated = true;
      }
    }
    bool is_qCurrent_updated = false;
    if (this->m_qCurrentIn.isNew() ) {
      while (this->m_qCurrentIn.isNew() ) this->m_qCurrentIn.read();
      if(this->m_qCurrent.data.length() == this->m_robot->numJoints()){
        for(int i=0;i<this->m_robot->numJoints();i++) this->m_qCurrentv[i] = this->m_qCurrent.data[i];
        is_qCurrent_updated = true;
      }
    }
    if(!is_qRef_updated || !is_qCurrent_updated){
      return RTC::RTC_OK;  // qRef, qCurrent が届かなければ何もしない
    }
    if (this->m_servoStateIn.isNew()) {
      while (this->m_servoStateIn.isNew()) this->m_servoStateIn.read();
        for (int i = 0; i < this->m_robot->numJoints(); i++ ){
          this->m_servoStatev[i] = (this->m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
        }
    }

    bool isCollision = false;

    if ( !this->m_enable ) {
      isCollision = false;
      this->m_collisionFreeOnce = false;
    }else{

      // check servo
      bool has_servoOn = false;
      for (int i = 0; i < this->m_robot->numJoints(); i++ ){
        has_servoOn = has_servoOn || this->m_servoStatev[i];
      }

      if(!has_servoOn){
        isCollision = false;
        this->m_collisionFreeOnce = false;

      }else{

        // qCurrentで干渉チェック
        bool currentCollision = false;
        {
          for ( int i = 0; i < this->m_robot->numJoints(); i++ ){
            this->m_robot->joint(i)->q() = this->m_qCurrentv[i];
          }
          this->m_robot->calcForwardKinematics();
          std::map<std::string, std::shared_ptr<CollisionLinkPair> >::iterator it = this->m_pair.begin();
          for ( int i = 0; it != m_pair.end(); it++, i++){
            std::shared_ptr<CollisionLinkPair> c = it->second;
            cnoid::Vector3 point0_local, point1_local;
            choreonoid_vclip::computeDistance(this->m_VclipLinks[c->link0], c->link0->p(), c->link0->R(),
                                              this->m_VclipLinks[c->link1], c->link1->p(), c->link1->R(),
                                              c->currentDistance, point0_local, point1_local);
            if(c->currentDistance <= this->tolerance) currentCollision = true;
          }
        }

        if(!currentCollision) {
          if(!this->m_collisionFreeOnce){
            this->m_collisionFreeOnce = true;
            std::cerr << "[" << this->m_profile.instance_name << "] [" << this->m_qRef.tm << "] set safe posture" << std::endl;
          }
        }

        if(!this->m_collisionFreeOnce){
          isCollision = false;

        }else{

          if(!currentCollision){
            isCollision = false;
          }else{
            // qRefへ向かう姿勢で干渉チェック
            bool getWorse = false;
            {
              cnoid::VectorX qNextv = this->m_qCurrentv + (this->m_qRefv - this->m_qCurrentv) * dt / recover_time;
              for ( int i = 0; i < this->m_robot->numJoints(); i++ ){
                this->m_robot->joint(i)->q() = qNextv[i];
              }
              this->m_robot->calcForwardKinematics();
              std::map<std::string, std::shared_ptr<CollisionLinkPair> >::iterator it = this->m_pair.begin();
              for ( int i = 0; it != m_pair.end(); it++, i++){
                std::shared_ptr<CollisionLinkPair> c = it->second;
                cnoid::Vector3 point0_local, point1_local;
                choreonoid_vclip::computeDistance(this->m_VclipLinks[c->link0], c->link0->p(), c->link0->R(),
                                                  this->m_VclipLinks[c->link1], c->link1->p(), c->link1->R(),
                                                  c->nextDistance, point0_local, point1_local);
                if(c->currentDistance <= this->tolerance) {
                  if(c->nextDistance <= c->currentDistance) {
                    getWorse = true;
                    if ( this->loop%200==0 || !this->prevCollision ) {
                      std::cerr << "[" << m_profile.instance_name << "] " << i << "/" << this->m_pair.size() << " pair: " << c->link0->name() << "/" << c->link1->name() << ", distance = " << c->currentDistance << " (" << c->nextDistance << ")" << std::endl;
                    }
                  }
                }else{
                  if(c->nextDistance <= this->tolerance) {
                    getWorse = true;
                    if ( this->loop%200==0 || !this->prevCollision ) {
                      std::cerr << "[" << m_profile.instance_name << "] " << i << "/" << this->m_pair.size() << " pair: " << c->link0->name() << "/" << c->link1->name() << ", distance = " << c->currentDistance << " (" << c->nextDistance << ")" << std::endl;
                    }
                  }
                }
              }
            }

            if(getWorse) isCollision = true;
            else isCollision = false;
          }
        }
      }

      // mode に応じて出力
      this->m_stopSignal.tm = m_qRef.tm;
      this->m_releaseSignal.tm = m_qRef.tm;
      if(isCollision){
        this->m_stopSignal.data = 1;
        this->m_releaseSignal.data = 0;
      }else{
        this->m_stopSignal.data = 0;
        this->m_releaseSignal.data = 1;
      }
      this->m_stopSignalOut.write();
      this->m_releaseSignalOut.write();

      this->prevCollision = isCollision;

    }
    return RTC::RTC_OK;
}



void CollisionDetector2::setupVClipModel(cnoid::BodyPtr i_body)
{
  for (unsigned int i=0; i<i_body->numLinks(); i++) {
    this->m_VclipLinks[i_body->link(i)] = choreonoid_vclip::convertToVClipModel(i_body->link(i)->collisionShape());
  }
}


bool CollisionDetector2::enable(void)
{
    if (m_enable){
        std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 is already enabled." << std::endl;
        return true;
    }
    std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 is successfully enabled." << std::endl;
    m_enable = true;
    return true;
}

bool CollisionDetector2::disable(void)
{
    if (!m_enable){
        std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 is already disabled." << std::endl;
        return true;
    }
    std::cerr << "[" << m_profile.instance_name << "] CollisionDetector2 is successfully disabled." << std::endl;
    m_enable = false;
    return true;
}

bool CollisionDetector2::setCollisionDetector2Param(const hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  this->tolerance = std::max(i_param.tolerance, 1e-4); // 完全にめりこむと距離計算が正しく機能しないので、マージンが必要
  this->recover_time = std::max(i_param.recover_time, 0.001);
  return true;
}

bool CollisionDetector2::getCollisionDetector2Param(hrpsys_ext_rtc::CollisionDetector2Service::CollisionDetector2Param& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  i_param.tolerance = this->tolerance;
  i_param.recover_time = this->recover_time;
  return true;
}

extern "C"
{

    void CollisionDetector2Init(RTC::Manager* manager)
    {
        RTC::Properties profile(component_spec);
        manager->registerFactory(profile,
                                 RTC::Create<CollisionDetector2>,
                                 RTC::Delete<CollisionDetector2>);
    }

};


