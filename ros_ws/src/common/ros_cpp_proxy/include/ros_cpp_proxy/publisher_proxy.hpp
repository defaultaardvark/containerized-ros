#ifndef _PUBLISHER_PROXY__HPP_
#define _PUBLISHER_PROXY__HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

namespace ros_proxy
{
template <typename MsgType>
class PublisherProxy
{
    public:
        PublisherProxy(const rclcpp::NodeOptions& options);

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
        {
            return node_->get_node_base_interface();
        }

    protected:
        virtual void populateMsg()
        {
        }

        virtual void setupAdditionalParameters()
        {
        }

        void initRos();
        void setupParameters();
        void publishMsg();

        rclcpp::Node::SharedPtr node_;

        MsgType msg_;
        std::string topic_;
        bool do_timer_;

    private:

        std::string node_name_;
        rclcpp::CallbackGroup::SharedPtr publisher_cb_group_, publisher_timer_cb_group_;
        rclcpp::TimerBase::SharedPtr publisher_timer_;
        std::shared_ptr<rclcpp::Publisher<MsgType>> publisher_;

};

template <typename MsgType>
PublisherProxy<MsgType>::PublisherProxy(const rclcpp::NodeOptions& options)
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
void PublisherProxy<MsgType>::setupParameters()
{
    rcl_interfaces::msg::ParameterDescriptor topic_desc;
    topic_desc.name = "publisher_topic";
    topic_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
    topic_ = node_->declare_parameter(topic_desc.name, "", topic_desc);

    rcl_interfaces::msg::ParameterDescriptor do_timer_desc;
    do_timer_desc.name = "do_timer";
    do_timer_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
    do_timer_ = node_->declare_parameter(do_timer_desc.name, false, do_timer_desc);
}

template <typename MsgType>
void PublisherProxy<MsgType>::initRos()
{
    publisher_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions publisher_options;
    publisher_options.callback_group = publisher_cb_group_;
    publisher_ = node_->create_publisher<MsgType>(topic_, 1, publisher_options);

    if (do_timer_)
    {
        publisher_timer_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        publisher_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(1.), std::bind(&PublisherProxy<MsgType>::publishMsg, this), publisher_timer_cb_group_);
    }
}

template <typename MsgType>
void PublisherProxy<MsgType>::publishMsg()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "data: " << msg_.data);
    publisher_->publish(msg_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "haha peepo");
}
} // namespace ros_proxy

#endif
