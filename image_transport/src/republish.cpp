/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"

#include "image_transport/image_transport.hpp"
#include "image_transport/publisher_plugin.hpp"


class ImageRepublisher : public rclcpp::Node 
{
  private:

  std::string in_topic, out_topic, in_transport, out_transport;
  bool params_changed;

  public:
    ImageRepublisher(const std::string & node_name = "image_republisher", const std::string & namespace_="")
    : Node(node_name, namespace_),
    params_changed(false)
    {
      init_params();
    };

    virtual ~ImageRepublisher()
    {

    };

    void init_params()
    {

      rcl_interfaces::msg::ParameterDescriptor topic_in;
      topic_in.name = "in_topic";
      topic_in.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      topic_in.description = "input topic name to be republished in a different transport ";
      topic_in.read_only = false;
      in_topic = declare_parameter("width","in",topic_in);

      rcl_interfaces::msg::ParameterDescriptor topic_out;
      topic_out.name = "out_topic";
      topic_out.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      topic_out.description = "ouput topic name of node ";
      topic_out.read_only = false;
      out_topic = declare_parameter("topic_out","image_republish_out",topic_out);


      rcl_interfaces::msg::ParameterDescriptor transport_in;
      transport_in.name = "in_transport";
      transport_in.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      transport_in.description = "name of transport for input topic, i.e, compressed, raw, ffmpeg";
      transport_in.read_only = false;
      in_transport= declare_parameter("transport_in","raw",transport_in);

      rcl_interfaces::msg::ParameterDescriptor transport_out; 
      transport_out.name = "out_transport";
      transport_out.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      transport_out.description = "name of transport for output topic, i.e, compressed, raw, ffmpeg";
      transport_out.read_only = false;
      out_transport= declare_parameter("transport_out","raw",transport_out);

      auto callback = std::bind(&ImageRepublisher::ParameterChangeCallback,this,std::placeholders::_1);
      this->set_on_parameters_set_callback(callback); 

    }


    rcl_interfaces::msg::SetParametersResult ParameterChangeCallback(const std::vector<rclcpp::Parameter> &params)
    {
      RCLCPP_INFO(get_logger(),"parameter change callback");
      in_transport = get_parameter("in_transport").value_to_string();
      out_transport = get_parameter("out_transport").value_to_string();
      in_topic = get_parameter("in_topic").value_to_string();
      out_topic= get_parameter("out_topic").value_to_string();
      params_changed = true;
    
    }

    bool get_params_changed()
    {
      return params_changed;
    }


};

int main(int argc, char ** argv)
{
  auto new_args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  auto node = std::make_shared<ImageRepublisher>();
  std::string in_transport, in_topic, out_transport, out_topic;

  if(!node->get_params_changed())
  {
   
    RCLCPP_INFO(node->get_logger(),"Command line args detected: Usage for image_republish: ros2 run image_transport image_republish <in_transport> <in_base_topic> <out_transport> <out_base_topic>");
    in_transport = new_args[1];
    in_topic = new_args[2];
    out_transport = new_args[3];
    out_topic = new_args[4];
  }
  else
  {

    in_transport = node->get_parameter("in_transport").as_string();
    in_topic = node->get_parameter("in_topic").as_string();
    out_transport = node->get_parameter("out_transport").as_string();
    out_topic = node->get_parameter("out_topic").as_string();
  }

  RCLCPP_INFO(node->get_logger(),"***********************Republish Info*********************************");
  RCLCPP_INFO(node->get_logger(),"in topic: %s",in_topic.c_str());
  RCLCPP_INFO(node->get_logger(),"in transport: %s",in_transport.c_str());
  RCLCPP_INFO(node->get_logger(),"out topic: %s",out_topic.c_str());
  RCLCPP_INFO(node->get_logger(),"out transport: %s",out_transport.c_str());

    // Load transport plugin
  typedef image_transport::PublisherPlugin Plugin;
  pluginlib::ClassLoader<Plugin> loader("image_transport", "image_transport::PublisherPlugin");
  std::string lookup_name = Plugin::getLookupName(out_transport.c_str());

  auto instance = loader.createUniqueInstance(lookup_name);
  std::shared_ptr<Plugin> pub = std::move(instance);
  pub->advertise(node.get(), out_topic.c_str());

  // Use PublisherPlugin::publish as the subscriber callback
  typedef void (Plugin::* PublishMemFn)(const sensor_msgs::msg::Image::ConstSharedPtr &) const;
  PublishMemFn pub_mem_fn = &Plugin::publishPtr;
  std::vector<rclcpp::TopicEndpointInfo> endpoints = node->get_publishers_info_by_topic(in_topic.c_str());

  rmw_qos_profile_t profile;
   if(endpoints.empty())
       {
         profile = rmw_qos_profile_sensor_data;
       }
    else
    {
      profile = endpoints.front().qos_profile().get_rmw_qos_profile();
    }
  RCLCPP_INFO(node->get_logger(),"***********************Republish Info*********************************");
  auto sub =
  image_transport::create_subscription(node.get(), in_topic.c_str(),
        std::bind(pub_mem_fn, pub.get(), std::placeholders::_1), in_transport.c_str(), profile);
  
  
  while(rclcpp::ok())
  {
  rclcpp::spin(node);
  }
  return 0;
}
