#ifndef LUNABOT_BEHAVIOR_BT_NODE_H
#define LUNABOT_BEHAVIOR_BT_NODE_H

#include <behaviortree_cpp_v3/bt_factory.h>
#include <iostream>
#include <ros/ros.h>

/*
    Tutorials pass in BT::PortsList literals in the providedPorts return statement. 
    The following constants below facilitates easier editing of input and output
    ports in one place 
*/
const BT::PortsList NAVIGATE_PORTS = { 
    BT::InputPort<std::string>("Where"), 
    BT::InputPort<std::string>("Input"), 
    BT::OutputPort<std::string>("Returns") 
};

const BT::PortsList EXCAVATE_PORTS = { 
    BT::InputPort<std::string>("message"), 
    BT::OutputPort<std::string>("Returns") 
};

const BT::PortsList DETECT_APRILTAG_PORTS = { 
    BT::InputPort<std::string>("message")  
};

const BT::PortsList DEPOSITION_PORTS = { 
    BT::InputPort<std::string>("message"),  
    BT::OutputPort<std::string>("Returns")  
};

class Navigate : public BT::SyncActionNode {
public:
    Navigate(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

};

class DetectApriltag : public BT::SyncActionNode {
public:
    DetectApriltag(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};


class Excavate : public BT::SyncActionNode {
    public:
    Excavate(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
    
};

class RunDeposition : public BT::SyncActionNode {
public:
    RunDeposition(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
    
};


#endif