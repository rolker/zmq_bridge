#include "zmq_bridge/zmq_bridge.h"
#include "std_msgs/String.h"
#include "geographic_msgs/GeoPointStamped.h"
 
std::shared_ptr<zmq::socket_t> publisher; 

int main(int argc, char **argv)
{
    zmq::context_t context(1);
    publisher = std::shared_ptr<zmq::socket_t>(new zmq::socket_t(context, ZMQ_PUB));
    publisher->bind("tcp://*:4200");
    
    ros::init(argc, argv, "zmg_bridge_agent");
    ros::NodeHandle n;
    
    zmq_bridge::ZMQPublisher zpub(publisher);
    
    ros::Subscriber position_sub = n.subscribe("/position",10, &zmq_bridge::ZMQPublisher::ROSCallback<geographic_msgs::GeoPointStamped, zmq_bridge::position>, &zpub);
    ros::Subscriber appcast_sub = n.subscribe("/moos/appcast",10,&zmq_bridge::ZMQPublisher::ROSCallback<std_msgs::String, zmq_bridge::appcast>, &zpub);
    
    while(ros::ok())
    {
        ros::spinOnce();
    }
    
    publisher->close();
    
    
    return 0;
}

