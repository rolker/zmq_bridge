#include <zmq.hpp>
#include "ros/ros.h"
#include "geographic_msgs/GeoPointStamped.h"

ros::Publisher pub;


int main(int argc, char **argv)
{
    zmq::context_t context(1);
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://192.168.56.2:4200");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);
    
    ros::init(argc, argv, "zmq_bridge_supervisor");
    ros::NodeHandle n;
    
    pub = n.advertise<geographic_msgs::GeoPointStamped>("/position",10);

    while(true)
    {
        zmq::message_t update;
        subscriber.recv(&update);
        
        geographic_msgs::GeoPointStamped gps;
        ros::serialization::IStream stream(reinterpret_cast<uint8_t*>(update.data()),update.size());
        ros::serialization::deserialize(stream,gps);
        
        pub.publish(gps);
        
    }
    
    return 0;
}
