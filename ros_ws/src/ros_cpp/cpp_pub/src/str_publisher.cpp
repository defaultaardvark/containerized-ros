#include "cpp_pub/str_publisher.hpp"

namespace publisher
{
    StrPublisher::StrPublisher(const rclcpp::NodeOptions& options) : StrPublisherProxy(options)
    {
        setupAdditionalParameters();
        populateMsg();
    }

    StrPublisher::~StrPublisher()
    {
    }

    void StrPublisher::setupAdditionalParameters()
    {
        rcl_interfaces::msg::ParameterDescriptor str_data_desc;
        str_data_desc.name = "str_data";
        str_data_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
        str_data_ = node_->declare_parameter(str_data_desc.name, "default_data", str_data_desc);
    }

    void StrPublisher::populateMsg()
    {
        msg_.data = str_data_;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(publisher::StrPublisher)
