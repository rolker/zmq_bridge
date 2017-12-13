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
        appcast,
        active,
        wpt_updates,
        origin,
	helm_mode,
        vehicle_status
    };

    class ZMQROSNode
    {
    public:
        ZMQROSNode(std::shared_ptr<zmq::socket_t> zmq_socket, std::shared_ptr<ros::NodeHandle> ros_node)
        :m_zmq_socket(zmq_socket),m_ros_node(ros_node)
        {
        }
        
    protected:
        std::shared_ptr<zmq::socket_t> m_zmq_socket;
        std::shared_ptr<ros::NodeHandle> m_ros_node;
    };
    
    class ZMQPublisher: public ZMQROSNode
    {
    public:
        ZMQPublisher(std::shared_ptr<zmq::socket_t> publisher, std::shared_ptr<ros::NodeHandle> ros_node):ZMQROSNode(publisher,ros_node)
        {
        }
    
        
        template<typename ROS_TYPE, Channel C> void addROSSubscriber(std::string const &topic)
        {
            m_ros_subscribers[C] = m_ros_node->subscribe(topic,10, &ZMQPublisher::ROSCallback<ROS_TYPE, C>, this);
        }
        
    private:
        template<typename ROS_TYPE, Channel C> void ROSCallback(const typename ROS_TYPE::ConstPtr& inmsg)
        {
            zmq::message_t channel_message(sizeof(Channel));
            uint32_t channel = C;
            memcpy(channel_message.data(),&channel,sizeof(channel));
            m_zmq_socket->send(channel_message,ZMQ_SNDMORE);
            
            uint32_t serial_size = ros::serialization::serializationLength(*inmsg);
            boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
            ros::serialization::OStream stream(buffer.get(), serial_size);
            ros::serialization::serialize(stream,*inmsg);
            zmq::message_t message(serial_size);
            memcpy(message.data(),buffer.get(),serial_size);
            m_zmq_socket->send(message);
        }

        typedef std::map<Channel,ros::Subscriber> SubscriberMap;
        SubscriberMap m_ros_subscribers;
    };
    
 
    
    class ZMQSubscriber:public ZMQROSNode
    {
    public:
        ZMQSubscriber(std::shared_ptr<zmq::socket_t> subscriber, std::shared_ptr<ros::NodeHandle> ros_node):ZMQROSNode(subscriber,ros_node)
        {
        }
        
        template<typename ROS_TYPE> void addROSPublisher(std::string const &topic, Channel channel)
        {
            m_ros_publishers[channel].rpub = m_ros_node->advertise<ROS_TYPE>(topic,10);
            m_ros_publishers[channel].decoder = &ZMQSubscriber::Decode<ROS_TYPE>;
        }
        
        void spinOnce()
        {
            zmq::message_t channel_message;
            if(m_zmq_socket->recv(&channel_message,ZMQ_NOBLOCK))
            {
                Channel c = *static_cast<Channel*>(channel_message.data());
                
                zmq::message_t data_message;
                m_zmq_socket->recv(&data_message);
                
                if(m_ros_publishers.find(c) != m_ros_publishers.end())
                    m_ros_publishers[c].decoder(data_message,m_ros_publishers[c].rpub);
            }
        }
        
        
    private:
        template<typename ROS_TYPE> static void Decode(zmq::message_t &message, ros::Publisher &pub)
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

        PublisherMap m_ros_publishers;
    };
}

#endif
