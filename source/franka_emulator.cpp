#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
	class FrankaEmulatorPlugin : public ModelPlugin
	{
    private:
        gazebo::physics::ModelPtr _model;
	public:
		FrankaEmulatorPlugin();
		virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
	};

	GZ_REGISTER_MODEL_PLUGIN(FrankaEmulatorPlugin)
}

gazebo::FrankaEmulatorPlugin::FrankaEmulatorPlugin()
{
}

void gazebo::FrankaEmulatorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
    _model = model;
    model->GetJoint("panda::panda_joint1")->SetPosition(0, 0.0);
    model->GetJoint("panda::panda_joint2")->SetPosition(0, -M_PI / 4);
    model->GetJoint("panda::panda_joint3")->SetPosition(0, 0.0);
    model->GetJoint("panda::panda_joint4")->SetPosition(0, -3 * M_PI / 4);
    model->GetJoint("panda::panda_joint5")->SetPosition(0, 0.0);
    model->GetJoint("panda::panda_joint6")->SetPosition(0, M_PI / 2);
    model->GetJoint("panda::panda_joint7")->SetPosition(0, M_PI / 4);
    model->GetJoint("panda::panda_finger_joint1")->SetPosition(0, 0.0);
    model->GetJoint("panda::panda_finger_joint2")->SetPosition(0, 0.0);
    
    std::cerr << "franka_emulator_plugin loaded" << std::endl;
}
