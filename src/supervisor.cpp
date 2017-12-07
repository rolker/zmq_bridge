#include "zmq_bridge/zmq_bridge.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geographic_msgs/GeoPointStamped.h"

int main(int argc, char **argv)
{
    zmq::context_t context(1);
    
    std::shared_ptr<zmq::socket_t>  subscriber(new zmq::socket_t(context, ZMQ_SUB));
    //subscriber.connect("tcp://192.168.56.2:4200");
    subscriber->connect("tcp://localhost:4200");
    subscriber->setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);
    
    std::shared_ptr<zmq::socket_t> publisher(new zmq::socket_t(context, ZMQ_PUB));
    publisher->connect("tcp://localhost:4201");
    
    ros::init(argc, argv, "zmq_bridge_supervisor");
    std::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle);
    
    zmq_bridge::ZMQSubscriber zsub(subscriber, n);
    zsub.addROSPublisher<geographic_msgs::GeoPointStamped>("/zmq/position",zmq_bridge::position);
    zsub.addROSPublisher<std_msgs::String>("/zmq/appcast",zmq_bridge::appcast);
    
    zmq_bridge::ZMQPublisher zpub(publisher, n);
    zpub.addROSSubscriber<std_msgs::Bool, zmq_bridge::active>("zmq/active");
    
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
