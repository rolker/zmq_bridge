#include "zmq_bridge/zmq_bridge.h"
#include "std_msgs/String.h"
#include "geographic_msgs/GeoPointStamped.h"

int main(int argc, char **argv)
{
    zmq::context_t context(1);
    zmq::socket_t subscriber (context, ZMQ_SUB);
    //subscriber.connect("tcp://192.168.56.2:4200");
    subscriber.connect("tcp://localhost:4200");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);
    
    ros::init(argc, argv, "zmq_bridge_supervisor");
    ros::NodeHandle n;
    
    zmq_bridge::PublisherMap pmap;

    pmap[zmq_bridge::position].rpub = n.advertise<geographic_msgs::GeoPointStamped>("/zmq/position",10);
    pmap[zmq_bridge::position].decoder = &zmq_bridge::Decode<geographic_msgs::GeoPointStamped>;

    pmap[zmq_bridge::appcast].rpub = n.advertise<std_msgs::String>("/zmq/appcast",10);
    pmap[zmq_bridge::appcast].decoder = &zmq_bridge::Decode<std_msgs::String>;
    
    while(ros::ok())
    {
        zmq::message_t channel_message;
        subscriber.recv(&channel_message);
        
        std::cerr << "c size: " << channel_message.size() << std::endl;

        zmq_bridge::Channel c = *static_cast<zmq_bridge::Channel*>(channel_message.data());
        
        zmq::message_t data_message;
        subscriber.recv(&data_message);
        
        std::cerr << "data size: " << data_message.size() << std::endl;
        
        if(pmap.find(c) != pmap.end())
            pmap[c].decoder(data_message,pmap[c].rpub);
    }
    
    return 0;
}
