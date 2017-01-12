#include <string>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>


namespace gazebo_dvl {
  class GazeboDVL : public gazebo::ModelPlugin {
    public:
      GazeboDVL(){};
      ~GazeboDVL(){};
      virtual void Load(gazebo::physics::ModelPtr _model,
                        sdf::ElementPtr _sdf);


    private:
      gazebo::physics::ModelPtr model_;
      gazebo::physics::LinkPtr link_;
      std::vector<gazebo::sensors::RaySensorPtr> rays_;

      sdf::ElementPtr getElement(sdf::ElementPtr element_parent,
                        std::string element_type,
                        std::string element_attribute,
                        std::string attribute_value);

      gazebo::physics::LinkPtr loadDVL(
                            gazebo::physics::ModelPtr model,
                            std::vector<gazebo::sensors::RaySensorPtr> &rays);
  };
  GZ_REGISTER_MODEL_PLUGIN(GazeboDVL)
}
