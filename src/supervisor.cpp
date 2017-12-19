#include "zmq_bridge/zmq_bridge.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "asv_msgs/VehicleStatus.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time.hpp"

std::map<zmq_bridge::Channel,std::string> zmq_bridge::ZMQROSNode::topic_map;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "zmq_bridge_supervisor");

    std::string host = "localhost";
    if (argc > 1)
        host = argv[1];
        
    zmq::context_t context(1);
    
    std::shared_ptr<zmq::socket_t>  subscriber(new zmq::socket_t(context, ZMQ_SUB));
    //subscriber.connect("tcp://192.168.56.2:4200");
    subscriber->connect("tcp://"+host+":4200");
    subscriber->setsockopt(ZMQ_SUBSCRIBE, nullptr, 0);
    
    std::shared_ptr<zmq::socket_t> publisher(new zmq::socket_t(context, ZMQ_PUB));
    publisher->setsockopt(ZMQ_SNDHWM, 20);
    publisher->connect("tcp://"+host+":4201");
    
    std::shared_ptr<ros::NodeHandle> n(new ros::NodeHandle);

    boost::posix_time::ptime now = ros::WallTime::now().toBoost();
    std::string iso_now = std::regex_replace(boost::posix_time::to_iso_extended_string(now),std::regex(":"),"-");
    
    std::string log_filename = "nodes/zmq_bridge_supervisor_receive-"+iso_now+".bag";
    
    zmq_bridge::ZMQSubscriber zsub(subscriber, n);
    zsub.openBag(log_filename);
    zsub.addROSPublisher<geographic_msgs::GeoPointStamped>("/zmq/position",zmq_bridge::position);
    zsub.addROSPublisher<std_msgs::String>("/zmq/appcast",zmq_bridge::appcast);
    zsub.addROSPublisher<geographic_msgs::GeoPoint>("/zmq/origin",zmq_bridge::origin);
    zsub.addROSPublisher<asv_msgs::VehicleStatus>("/zmq/vehicle_status",zmq_bridge::vehicle_status);

    log_filename = "nodes/zmq_bridge_supervisor_send-"+iso_now+".bag";
    
    zmq_bridge::ZMQPublisher zpub(publisher, n);
    zpub.openBag(log_filename);
    zpub.addROSSubscriber<std_msgs::Bool, zmq_bridge::active>("zmq/active");
    zpub.addROSSubscriber<std_msgs::String, zmq_bridge::helm_mode>("zmq/helm_mode");
    zpub.addROSSubscriber<std_msgs::String, zmq_bridge::wpt_updates>("zmq/wpt_updates");
    zpub.addROSSubscriber<std_msgs::String, zmq_bridge::loiter_updates>("zmq/loiter_updates");
    
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
