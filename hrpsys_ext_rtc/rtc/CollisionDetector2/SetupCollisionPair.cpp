#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <cnoid/BodyLoader>
#include <cnoid/JointPath>
#include <choreonoid_qhull/choreonoid_qhull.h>
#include <choreonoid_vclip/choreonoid_vclip.h>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

bool checkCollisionForAllJointRange(int i, cnoid::JointPath& jointPath, const std::vector<std::pair<cnoid::LinkPtr,cnoid::LinkPtr> >&collisionPairs, std::vector<bool>& always_collide, std::vector<bool>& never_collide, std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> >& vclipLinks){
  if ( i >= jointPath.numJoints() ) {
    jointPath.joint(0)->body()->calcForwardKinematics();
    for(int j=0;j<collisionPairs.size();j++){
      if(!always_collide[j] && !never_collide[j]) continue;
      cnoid::LinkPtr A_link = collisionPairs[j].first;
      cnoid::LinkPtr B_link = collisionPairs[j].second;
      cnoid::Vector3 A_localp, B_localp;
      double dist;
      bool solved = choreonoid_vclip::computeDistance(vclipLinks[A_link],
                                                      A_link->p(),
                                                      A_link->R(),
                                                      vclipLinks[B_link],
                                                      B_link->p(),
                                                      B_link->R(),
                                                      dist,
                                                      A_localp,
                                                      B_localp
                                                      );
      if(dist > 0) always_collide[j] = false;
      else never_collide[j] = false;
    }
    return true;
  }

  cnoid::LinkPtr l = jointPath.joint(i);

  if(l->isRevoluteJoint() || l->isPrismaticJoint()){
    double step = (l->q_upper() - l->q_lower())/2;
    for(double angle = l->q_lower(); angle <= l->q_upper(); angle += step){
      l->q() = angle;
      checkCollisionForAllJointRange(i+1, jointPath, collisionPairs, always_collide, never_collide, vclipLinks);
    }
  }else{
    checkCollisionForAllJointRange(i+1, jointPath, collisionPairs, always_collide, never_collide, vclipLinks);
  }
  return true;
}

int main (int argc, char** argv)
{
    std::string url;
    int path;
    std::unordered_set<std::string> blacklist;
    std::vector<std::unordered_set<std::string> > blacklistGroups;

    {
      std::vector<std::string> raw_blacklist;
      std::vector<std::string> raw_blacklistGroups;
      boost::program_options::options_description desc("Options");
      desc.add_options()
        ("model",
         boost::program_options::value<std::string>(&url),
         "model file name");
      desc.add_options()
        ("path",
         boost::program_options::value<int>(&path)->default_value(6),
         "maximum path length");
      desc.add_options()
        ("blacklist",
         boost::program_options::value<std::vector<std::string> >(&raw_blacklist)->multitoken()->composing(),
         "R_HAND_J0 L_HAND_J0 EYEBROW_P EYELID_P:EYE_Y");
      desc.add_options()
        ("blacklistgroup",
         boost::program_options::value<std::vector<std::string> >(&raw_blacklistGroups)->multitoken()->composing(),
         "R_HAND_J0,L_HAND_J0,EYEBROW_P EYELID_P,EYE_Y,EYE_P,MOUTH_P UPPERLIP_P,LOWERLIP_P,CHEEK_P");
      boost::program_options::variables_map vm;
      boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
      boost::program_options::notify(vm);

      blacklist = std::unordered_set<std::string>(raw_blacklist.begin(),raw_blacklist.end());
      for(int i=0;i<raw_blacklistGroups.size();i++){
        std::vector<std::string> group;
        boost::split(group, raw_blacklistGroups[i], boost::is_any_of(","));
        blacklistGroups.emplace_back(group.begin(),group.end());
      }
    }

    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(url);
    if (!robot){
      std::cerr << "failed to load model[" << url << "]" << std::endl;
      return 1;
    }
    choreonoid_qhull::convertAllCollisionToConvexHull(robot);
    std::unordered_map<cnoid::LinkPtr, std::shared_ptr<Vclip::Polyhedron> > vclipLinks;
    for (int i=0; i<robot->numLinks(); i++) {
      vclipLinks[robot->link(i)] = choreonoid_vclip::convertToVClipModel(robot->link(i)->collisionShape());
    }

    std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > collisionPairs;

    std::cout << "Setup Initial collision pair without " << std::endl;
    for(std::unordered_set<std::string>::iterator it = blacklist.begin(); it != blacklist.end(); it++ ) {
      std::cout << *it << " ";
    }
    std::cout << std::endl;
    for(int i=0;i<blacklistGroups.size();i++){
      for(std::unordered_set<std::string>::iterator it = blacklistGroups[i].begin(); it != blacklistGroups[i].end(); it++ ) {
        std::cout << *it << " ";
      }
      std::cout << std::endl;
    }
    for (unsigned int i=0; i<robot->numLinks(); i++) {
      cnoid::LinkPtr l1 = robot->link(i);
      for (unsigned int j=i+1; j<robot->numLinks(); j++) {
        cnoid::LinkPtr l2 = robot->link(j);
        if ( l1->collisionShape()->numChildren() != 0 &&
             l2->collisionShape()->numChildren() != 0 &&
             blacklist.find(l1->name()) == blacklist.end() &&
             blacklist.find(l2->name()) == blacklist.end() &&
             blacklist.find(l1->name() + ":" + l2->name()) == blacklist.end() &&
             blacklist.find(l2->name() + ":" + l1->name()) == blacklist.end()){
          bool found = false;
          for(int g=0;g<blacklistGroups.size();g++){
            if(blacklistGroups[g].find(l1->name()) != blacklistGroups[g].end() &&
               blacklistGroups[g].find(l2->name()) != blacklistGroups[g].end()){
              found = true;
              break;
            }
          }
          if(!found){
            collisionPairs.emplace_back(l1, l2);
          }
        }
      }
    }
    std::cout << "Initial collision pair size " << collisionPairs.size() << std::endl;

    std::cout << "step 0: Remove collision pair if they are adjacent pair" << std::endl;
    {
      std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > nextCollisionPairs;
      nextCollisionPairs.reserve(collisionPairs.size());
      for(int i=0;i<collisionPairs.size();i++){
        cnoid::LinkPtr l1 = collisionPairs[i].first;
        cnoid::LinkPtr l2 = collisionPairs[i].second;
        cnoid::JointPath jointPath(l1,l2);
        if(jointPath.numJoints() == 1) {
          std::cout << "  pair (" << jointPath.numJoints() << ") " << l1->name() << "/" << l2->name() << std::endl;
        }else{
          nextCollisionPairs.push_back(collisionPairs[i]);
        }
      }
      collisionPairs = nextCollisionPairs;
    }
    std::cout << "collision pair size " << collisionPairs.size() << std::endl;

    std::cout << "step1:  Remove always/never collide pair for a length of 2..." << path << std::endl;
    {
      std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > nextCollisionPairs;
      nextCollisionPairs.reserve(collisionPairs.size());

      std::vector<std::pair<cnoid::LinkPtr, cnoid::LinkPtr> > checkCollisionPairs;
      checkCollisionPairs.reserve(collisionPairs.size());
      for(int i=0;i<collisionPairs.size();i++){
        cnoid::LinkPtr l1 = collisionPairs[i].first;
        cnoid::LinkPtr l2 = collisionPairs[i].second;
        cnoid::JointPath jointPath(l1,l2);
        if(jointPath.numJoints() <= path) {
          checkCollisionPairs.push_back(collisionPairs[i]);
        }else{
          nextCollisionPairs.push_back(collisionPairs[i]);
        }
      }

      // setup data structure for check collision pair
      // pair_tree[<pair:L1,L2,L3,L4>, [ <pair:L1,L2,L3,L4>, <pair:L1,L2,L3>, <pair:L1,L2> ]]
      // pair_tree[<pair:L2,L3,L4,L5>, [ <pair:L2,L3,L4,L5>, <pair:L2,L3,L4>, <pair:L2,L3> ]]
      std::vector<std::pair<std::pair<cnoid::LinkPtr,cnoid::LinkPtr>, std::vector<std::pair<cnoid::LinkPtr,cnoid::LinkPtr> > > > pair_tree;
      std::sort(checkCollisionPairs.begin(),
                checkCollisionPairs.end(),
                [](const std::pair<cnoid::LinkPtr,cnoid::LinkPtr>& p1, const std::pair<cnoid::LinkPtr,cnoid::LinkPtr>& p2){
                  return (cnoid::JointPath(p1.first,p1.second).numJoints()) > (cnoid::JointPath(p2.first,p2.second).numJoints());
                });
      for(int c=0;c<checkCollisionPairs.size();c++){
        cnoid::JointPath jointPath1(checkCollisionPairs[c].first,checkCollisionPairs[c].second);
        bool is_new_key = true;
        for (int i=0;i<pair_tree.size();i++) {
          cnoid::JointPath jointPath2(pair_tree[i].first.first,pair_tree[i].first.second);
          // check if JointPath1 is included in jointPath2
          bool find_key = true;
          for (unsigned int j = 0; j < jointPath1.numJoints() ; j++ ) {
            if ( jointPath1.joint(j) != jointPath2.joint(j) ) {
              find_key = false;
              break;
            }
          }
          if ( find_key ) {
            pair_tree[i].second.push_back(checkCollisionPairs[c]);
            is_new_key = false;
            break;
          }
        }
        if (is_new_key) {
          pair_tree.emplace_back(checkCollisionPairs[c],std::vector<std::pair<cnoid::LinkPtr,cnoid::LinkPtr> >{checkCollisionPairs[c]});
        }
      }

      for (int i=0;i<pair_tree.size();i++) {
        std::pair<cnoid::LinkPtr,cnoid::LinkPtr> key_pair = pair_tree[i].first;
        std::vector<std::pair<cnoid::LinkPtr,cnoid::LinkPtr> > sub_pairs = pair_tree[i].second;
        std::vector<bool> always_collide(sub_pairs.size(),true);
        std::vector<bool> never_collide(sub_pairs.size(),true);
        cnoid::JointPath jointPath(key_pair.first,key_pair.second);
        std::cout << "  pair (" << jointPath.numJoints() << ") " << key_pair.first->name() << "/" << key_pair.second->name() << " " << sub_pairs.size() << std::endl;
        // rmeove non-collision pair from sub_paris
        checkCollisionForAllJointRange(0, jointPath, sub_pairs, always_collide, never_collide, vclipLinks);
        for(int j=0;j<sub_pairs.size();j++){
          if(!always_collide[j] && !never_collide[j]) nextCollisionPairs.push_back(sub_pairs[j]);
        }
      }
      collisionPairs = nextCollisionPairs;
    }

    std::cout << "Reduced collision pair size " << collisionPairs.size() << std::endl;

    std::sort(collisionPairs.begin(),
              collisionPairs.end(),
              [](const std::pair<cnoid::LinkPtr,cnoid::LinkPtr>& p1, const std::pair<cnoid::LinkPtr,cnoid::LinkPtr>& p2){
                return
                  (p1.first->index() < p2.first->index()) ||
                  (p1.first->index() == p2.first->index() && p1.second->index() < p2.second->index());
              });

    std::string fname = "/tmp/"+robot->modelName()+"collision_pair.conf";
    std::ofstream ofs(fname);
    ofs << "collision_pair:";
    for (int i=0;i<collisionPairs.size();i++) {
      ofs << " " << collisionPairs[i].first->name() << ":" << collisionPairs[i].second->name();
      std::cout << collisionPairs[i].first->name() << ":" << collisionPairs[i].second->name() << std::endl;
    }
    ofs << std::endl;
    ofs.close();
    std::cout << "Write collision pair conf file to " << fname << std::endl;

    return 0;
}
