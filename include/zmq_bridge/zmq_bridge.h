#ifndef ZMQ_BRIDGE_H
#define ZMQ_BRIDGE_H

#include <memory>
#include <functional>
#include <zmq.hpp>
#include "ros/ros.h"

namespace zmq_bridge
{
    enum Channel : uint32_t
    {
        position,
        appcast
    };
    
    class ZMQPublisher
    {
    public:
        ZMQPublisher(std::shared_ptr<zmq::socket_t> publisher):m_publisher(publisher)
        {
        }
    
        template<typename ROS_TYPE, Channel C> void ROSCallback(const typename ROS_TYPE::ConstPtr& inmsg)
        {
            zmq::message_t channel_message(sizeof(Channel));
            uint32_t channel = C;
            memcpy(channel_message.data(),&channel,sizeof(channel));
            m_publisher->send(channel_message,ZMQ_SNDMORE);
            
            uint32_t serial_size = ros::serialization::serializationLength(*inmsg);
            boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
            ros::serialization::OStream stream(buffer.get(), serial_size);
            ros::serialization::serialize(stream,*inmsg);
            zmq::message_t message(serial_size);
            memcpy(message.data(),buffer.get(),serial_size);
            m_publisher->send(message);
        }
        
    private:
        std::shared_ptr<zmq::socket_t> m_publisher;
    };
    
    template<typename ROS_TYPE> void Decode(zmq::message_t &message, ros::Publisher &pub)
    {
        ROS_TYPE ros_msg;
        boost::shared_array<uint8_t> buffer(new uint8_t[message.size()]);
        memcpy(buffer.get(),message.data(),message.size());
        
        ros::serialization::IStream stream(buffer.get(),message.size());
        ros::serialization::Serializer<ROS_TYPE>::read(stream, ros_msg);
        
        pub.publish(ros_msg);
    }
    
    struct ROSPublisher
    {
        ros::Publisher rpub;
        void (*decoder)(zmq::message_t &, ros::Publisher &);
    };
    
    typedef std::map<Channel,ROSPublisher> PublisherMap;
}

#endif
