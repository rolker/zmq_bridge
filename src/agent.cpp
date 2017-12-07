#include "zmq_bridge/zmq_bridge.h"
#include "std_msgs/String.h"
#include "geographic_msgs/GeoPointStamped.h"
 

int main(int argc, char **argv)
{
    zmq::context_t context(1);
    std::shared_ptr<zmq::socket_t> publisher(new zmq::socket_t(context, ZMQ_PUB));
    publisher->bind("tcp://*:4200");
    
    ros::init(argc, argv, "zmg_bridge_agent");
    std::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle);
    
    zmq_bridge::ZMQPublisher zpub(publisher, n);
    
    zpub.addROSSubscriber<geographic_msgs::GeoPointStamped, zmq_bridge::position>("/position");
    zpub.addROSSubscriber<std_msgs::String, zmq_bridge::appcast>("/moos/appcast");
    
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    publisher->close();
    
    return 0;
}

