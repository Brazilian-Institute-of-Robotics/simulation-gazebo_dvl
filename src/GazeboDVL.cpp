#include "GazeboDVL.hpp"

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <iostream>
#include <sstream>

#define DVL_BEAMS 4

using namespace gazebo_dvl;

std::string HEADER_MSG = "GazeboDVL: ";
std::string LINK_NAME = "dvl";

template <typename T>
std::string numberToString ( T Number ){
  std::ostringstream ss;
  ss << Number;
  return ss.str();
}

void GazeboDVL::Load(gazebo::physics::ModelPtr _model,
                     sdf::ElementPtr _sdf){

  model_ = _model;
  link_ = loadDVL(model_, rays_);

  // check required elements
  if(link_){
    if(rays_.size() == DVL_BEAMS)
      gzmsg << HEADER_MSG << "loaded DVL plugin successfully" << std::endl;
    else{
      gzerr << "DVL plugin requires " << DVL_BEAMS << " ray sensors" << std::endl;
      return;
    }
  }

}

sdf::ElementPtr GazeboDVL::getElement( sdf::ElementPtr element_parent,
                              std::string element_type,
                              std::string element_attribute,
                              std::string attribute_value){

  sdf::ElementPtr element;
  if(element_parent->HasElement(element_type)){
    element = element_parent->GetElement(element_type);
    while(element)
      if(element->Get<std::string>(element_attribute) == attribute_value){
        break;
      }else{
        element = element->GetNextElement(element_type);
      }
  }

  return element;
}

gazebo::physics::LinkPtr GazeboDVL::loadDVL(
                        gazebo::physics::ModelPtr model,
                        std::vector<gazebo::sensors::RaySensorPtr> &rays){

  gzmsg << HEADER_MSG + "loading DVL plugin from model: "
                        + model_->GetName() << std::endl;
  // check dvl link
  gazebo::physics::LinkPtr link_ptr = model_->GetLink(LINK_NAME);

  if(link_ptr)
    gzmsg << HEADER_MSG + "found link " + LINK_NAME << std::endl;
  else
    gzerr << "did not find link " << LINK_NAME << std::endl;

  // check ray sensors number
  bool check_rays = (bool) link_ptr;
  for (uint i = 0; (i < DVL_BEAMS) && check_rays ; i++){
    std::string sensor_name = link_ptr->GetSensorName(i);
    gazebo::sensors::SensorPtr sensor_ptr;
    sensor_ptr = gazebo::sensors::get_sensor(sensor_name);

    if(sensor_ptr){
      rays.push_back(
        std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(sensor_ptr));
      gzmsg <<  HEADER_MSG + "sensor found : " + sensor_name << std::endl;
    }else
      check_rays = false;
  }


  if(link_ptr){
    if(check_rays)
      gzmsg <<  HEADER_MSG + "found " + numberToString(DVL_BEAMS) +
        " beams ( ray sensor )" << std::endl;
    else
      gzerr <<  "did not found " + numberToString(DVL_BEAMS) +
          " beams ( ray sensor )" << std::endl;
  }

  return link_ptr;

}
