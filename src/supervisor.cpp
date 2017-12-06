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

    while(ros::ok())
    {
        zmq::message_t update;
        subscriber.recv(&update);
        
        std::cerr << "data size: " << update.size() << std::endl;
        
        geographic_msgs::GeoPointStamped gps;
        
        std::cerr << "serialization size: " <<  ros::serialization::serializationLength(gps) << std::endl;
        
        boost::shared_array<uint8_t> buffer(new uint8_t[update.size()]);
        memcpy(buffer.get(),update.data(),update.size());
        
        ros::serialization::IStream stream(buffer.get(),update.size());
        ros::serialization::Serializer<geographic_msgs::GeoPointStamped>::read(stream, gps);
        
        pub.publish(gps);
        
    }
    
    return 0;
}
