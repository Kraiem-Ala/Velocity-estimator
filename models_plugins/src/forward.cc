#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include <ros/ros.h>
#include<unistd.h>
#include <cmath>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->periode = 60.0 ;
       gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
          new gazebo::common::PoseAnimation("test", this->periode, true));


        if (_sdf->HasElement("goals"))
          {
            if (this->load_goals(_sdf->GetElement("goals")))
            {
              gazebo::common::PoseKeyFrame *key;
              float time = 0 ;
              int e = 0 ;
              key = anim->CreateKeyFrame(0);
              key->Translation(this->pathGoals[0].Pos());
              key->Rotation(this->pathGoals[0].Rot());
              e++ ;
              for (std::vector<ignition::math::Pose3d>::iterator i = this->pathGoals.begin() +1 ; i != this->pathGoals.end(); ++i)
              {
                
                float distance = abs(this->pathGoals[e].Pos().Distance(this->pathGoals[e-1].Pos()) ) ;                
                if (distance < 2)
                {
                  time += (distance / 0.1) ;
                  key = anim->CreateKeyFrame(time) ;
                  key->Rotation(this->pathGoals[e].Rot());
                  key->Translation(this->pathGoals[e].Pos());
                  
                }
                else 
                {
                  time += ( distance / 0.5 ) ; 
                  key = anim->CreateKeyFrame(time) ;
                  key->Rotation(this->pathGoals[e].Rot());
                  key->Translation(this->pathGoals[e].Pos());
                }
                e++ ;
              }
              anim->SetLength(time) ;
            // set the animation
            _parent->SetAnimation(anim);
            }
          }
       
        

       /* // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(1, -2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, -1.5707));

        // set waypoint location after 2 seconds

        key = anim->CreateKeyFrame(18.0);
        key->Translation(ignition::math::Vector3d(18, -2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, -1.5707));

        key = anim->CreateKeyFrame(20.0);
        key->Translation(ignition::math::Vector3d(18, -2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, -3.14));


        key = anim->CreateKeyFrame(28);
        key->Translation(ignition::math::Vector3d(18, -10, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, -3.14));

        key = anim->CreateKeyFrame(30.0);
        key->Translation(ignition::math::Vector3d(18, -10, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(38.0);
        key->Translation(ignition::math::Vector3d(18, -2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(42.0);
        key->Translation(ignition::math::Vector3d(18, -2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        key = anim->CreateKeyFrame(50);
        key->Translation(ignition::math::Vector3d(10, -2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        key = anim->CreateKeyFrame(56.0);
        key->Translation(ignition::math::Vector3d(1, -2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, 1.5707));

        // set final location equal to starting location
        key = anim->CreateKeyFrame(60);
        key->Translation(ignition::math::Vector3d(1, -2, 0));
        key->Rotation(ignition::math::Quaterniond(0, 0, -1.5707));*/


        
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      //this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      //    std::bind(&ModelPush::OnUpdate, this));
      
      ROS_WARN("Loaded ModelPush Plugin with parent...%s", this->model->GetName().c_str());
    }

    // Called by the world update start event

    public : bool load_goals(const sdf::ElementPtr _sdf)
    {
      gzmsg << "[model_move] Processing path goals defined in the SDF file" << std::endl;
      GZ_ASSERT(_sdf, "_sdf element is null");

      if (!_sdf->HasElement("pose"))
      {
        gzerr << "[model_move] SDF with goals tag but without pose/s element/s"<< std::endl;
        return false;
      }

      sdf::ElementPtr poseElem = _sdf->GetElement("pose");

      while (poseElem)
      {
        this->pathGoals.push_back(poseElem->Get<ignition::math::Pose3d>());
        poseElem = poseElem->GetNextElement("pose");
      }

      GZ_ASSERT(this->pathGoals.size() > 0, "pathGoals should not be zero");

      return true;
    }

    public : std::vector<ignition::math::Pose3d> pathGoals ;
    public : std::vector<ignition::math::Quaterniond> pathorientation ;
    public : float periode ;
    // Pointer to the model
    private: physics::ModelPtr model;
    

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}