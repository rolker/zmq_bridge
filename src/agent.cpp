#include "zmq_bridge/zmq_bridge.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geographic_msgs/GeoPointStamped.h"
 

int main(int argc, char **argv)
{
    zmq::context_t context(1);

    std::shared_ptr<zmq::socket_t> publisher(new zmq::socket_t(context, ZMQ_PUB));
    publisher->setsockopt(ZMQ_SNDHWM, 20);
    publisher->bind("tcp://*:4200");

    std::shared_ptr<zmq::socket_t>  subscriber(new zmq::socket_t(context, ZMQ_SUB));
    subscriber->bind("tcp://*:4201");
    subscriber->setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);
    
    ros::init(argc, argv, "zmg_bridge_agent");
    std::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle);
    
    zmq_bridge::ZMQPublisher zpub(publisher, n);
    
    zpub.addROSSubscriber<geographic_msgs::GeoPointStamped, zmq_bridge::position>("/position");
    zpub.addROSSubscriber<std_msgs::String, zmq_bridge::appcast>("/moos/appcast");
    zpub.addROSSubscriber<geographic_msgs::GeoPoint,zmq_bridge::origin>("/moos/origin");
    
    zmq_bridge::ZMQSubscriber zsub(subscriber, n);
    
    zsub.addROSPublisher<std_msgs::Bool>("/active",zmq_bridge::active);
    zsub.addROSPublisher<std_msgs::String>("/helm_mode",zmq_bridge::helm_mode);
    zsub.addROSPublisher<std_msgs::String>("/moos/wpt_updates",zmq_bridge::wpt_updates);
    
    while(ros::ok())
    {
        zsub.spinOnce();
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }
    
    publisher->close();
    subscriber->close();
    
    return 0;
}

