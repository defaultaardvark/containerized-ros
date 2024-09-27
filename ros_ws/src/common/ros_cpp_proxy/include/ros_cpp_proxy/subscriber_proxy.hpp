#ifndef _SUBSCRIBER_PROXY__HPP_
#define _SUBSCRIBER_PROXY__HPP_

#include <rclcpp/rclcpp.hpp>

namespace ros_proxy
{
template <typename MsgType>
class SubscriberProxy
{
    public:
        SubscriberProxy(const rclcpp::NodeOptions& options);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
        {
            return node_->get_node_base_interface();
        }

    protected:
        virtual void subscriberCallback()
        {
        }

        virtual void setupAdditionalParameters()
        {
        }

        void initRos();
        void setupParameters();

        rclcpp::Node::SharedPtr node_;

        MsgType msg_;
        std::string topic_;

    private:
        std::string node_name_;
        rclcpp::CallbackGroup::SharedPtr subscriber_cb_group_;
        std::shared_ptr<rclcpp::Subscriber<MsgType>> subscriber_;
};

template <typename MsgType>
SubscriberProxy<MsgType>::SubscriberProxy(const rclcpp::NodeOptions& options)
{
    auto arguments = options.arguments();
    for (const auto& arg : arguments)
    {
        if (arg.find("__node:=") != std::string::npos)
        {
            node_ = std::make_shared<rclcpp::Node>(arg.substr(8).c_str(), options);
        }
    }

    node_name_ = node_->get_name();

    setupParameters();
    initRos();
}

template <typename MsgType>
void SubscriberProxy<MsgType>::setupParameters()
{
    rcl_interfaces::msg::ParameterDescriptor topic_desc;
    topic_desc.name = "subscriber_topic";
    topic_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
    topic_ = node_->declare_parameter(topic_desc.name, "", topic_desc);
}

template <typename MsgType>
void SubscriberProxy<MsgType>::initRos()
{
    subscriber_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MututallyExclusive);

    rclcpp::SubscriptionOptions options;
    options.callback_group = subscriber_cb_group_;

    subscriber_ = node_->create_subscription<MsgType>(topic_, 1, std::bind(&SubscriberProxy::subscriberCallback, this), options);
}
} // namespace ros_proxy

#endif

