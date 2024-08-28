#ifndef _STR_PUBLISHER__HPP_
#define _STR_PUBLISHER__HPP_

#include <std_msgs/msg/string.hpp>

#include "ros_cpp_proxy/publisher_proxy.hpp"

namespace publisher
{
using MsgType = std_msgs::msg::String;
typedef ros_proxy::PublisherProxy<MsgType> StrPublisherProxy;

class StrPublisher : public StrPublisherProxy
{
    public:
        StrPublisher(const rclcpp::NodeOptions& options);
        virtual ~StrPublisher();

    protected:
        void setupAdditionalParameters() override;
        void populateMsg() override;

    private:
        std::string str_data_;

};
} // namespace publisher

#endif