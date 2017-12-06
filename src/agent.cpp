#include <zmq.hpp>
#include "ros/ros.h"
#include "geographic_msgs/GeoPointStamped.h"
 
zmq::socket_t *publisher; 
 
void positionCallback(const geographic_msgs::GeoPointStamped::ConstPtr& inmsg)
{
    uint32_t serial_size = ros::serialization::serializationLength(*inmsg);
    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
    ros::serialization::OStream stream(buffer.get(), serial_size);
    ros::serialization::serialize(stream,*inmsg);
    zmq::message_t message(serial_size);
    memcpy(message.data(),buffer.get(),serial_size);
    publisher->send(message);
}

int main(int argc, char **argv)
{
    zmq::context_t context(1);
    publisher = new zmq::socket_t(context, ZMQ_PUB);
    publisher->bind("tcp://*:4200");
    
    ros::init(argc, argv, "zmg_bridge_agent");
    ros::NodeHandle n;
    
    ros::Subscriber position_sub = n.subscribe("/position",10,positionCallback);
    
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    publisher->close();
    
    
    return 0;
}

