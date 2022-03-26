//#include "behaviortree_cpp_v3/bt_factory.h"
#include <lunabot_behavior/lunabot_bt_node.hpp>

RunDeposition::RunDeposition(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus RunDeposition::tick()
{
    BT::Optional<std::string> msg = getInput<std::string>("message");
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ", 
            msg.error() );
    }

    std::cout << "Robot is what? " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// A node having ports MUST implement this STATIC method
BT::PortsList RunDeposition::providedPorts()
{
    return DEPOSITION_PORTS;
}

Excavate::Excavate(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus Excavate::tick()
{
    BT::Optional<std::string> msg = getInput<std::string>("message");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ", 
            msg.error() );
    }

    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    setOutput("Returns", "Success!");
    return BT::NodeStatus::SUCCESS;
}

// A node having ports MUST implement this STATIC method
BT::PortsList Excavate::providedPorts()
{
    return EXCAVATE_PORTS;
}

Navigate::Navigate(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
}

// This Action simply write a value in the port "text"
BT::NodeStatus Navigate::tick()
{
    BT::Optional<std::string> msg = getInput<std::string>("Where");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ", 
            msg.error() );
    }

    // use the method value() to extract the valid message.
    std::cout << "Robot is going to the following location: " << msg.value() << std::endl;
    setOutput("Returns", "The answer is 42");
    return BT::NodeStatus::SUCCESS;
}

// A node having ports MUST implement this STATIC method
BT::PortsList Navigate::providedPorts()
{
    return NAVIGATE_PORTS;
}


BT::PortsList DetectApriltag::providedPorts()
{
        // This action has a single input port called "message"
        // Any port must have a name. The type is optional.
        return DETECT_APRILTAG_PORTS;
}

BT::NodeStatus DetectApriltag::tick()
{
    BT::Optional<std::string> msg = getInput<std::string>("message");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ", 
            msg.error() );
    }

    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

DetectApriltag::DetectApriltag(const std::string& name, const BT::NodeConfiguration& config) 
: SyncActionNode(name, config)
{    
} 

static const char* xml_text = R"(
<?xml version="1.0"?>
    <root main_tree_to_execute="BehaviorTree">
        <!-- ////////// -->
        <BehaviorTree ID="BehaviorTree">
            <ReactiveSequence name="Keep looking for apriltags until found, then start behavior">
                <Action ID="DetectApriltag" message="detecting an apriltag"/>
                <Sequence name="If apriltags found, start competition behaviors">
                    <Action ID="Navigate" Input="" Returns="SUCCESS/FAIL/ELSE" Where="Excavation"/>
                    <Action ID="Excavate" message="Digging a gravel" Returns="SUCCESS/FAIL/ELSE"/>
                    <Action ID="Navigate" Input="sieve_position" Returns="SUCCESS/FAIL/ELSE" Where="Bin"/>
                </Sequence>
            </ReactiveSequence>
        </BehaviorTree>
        <!-- ////////// -->
        <TreeNodesModel>
            <Action ID="DetectApriltag">
                <input_port default="detecting an apriltag" name="message"/>
            </Action>
            <Action ID="Excavate">
                <input_port name="message"/>
                <output_port default="SUCCESS/FAIL/ELSE" name="Returns"/>
            </Action>
            <Action ID="Navigate">
                <input_port name="Input"/>
                <output_port default="SUCCESS/FAIL/ELSE" name="Returns"/>
                <input_port name="Where"/>
            </Action>
        </TreeNodesModel>
        <!-- ////////// -->
    </root>
 )";


int main(int argc, char **argv) {
    ros::init(argc, argv, "test_behavior_tree");
    ros::NodeHandle nh;

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<DetectApriltag>("DetectApriltag");
    factory.registerNodeType<Navigate>("Navigate");
    factory.registerNodeType<Excavate>("Excavate");
    factory.registerNodeType<RunDeposition>("RunDeposition");

    //auto tree = factory.createTreeFromText(xml_text);

    // file path is temporary; will probably store literal to a const variable later with 
    // absolute filepath
    auto tree = factory.createTreeFromFile("src/purdue_lunabotics/lunabot_behavior/src/LUNABOT_BT_2.xml");
    
    tree.tickRoot();

    ros::spin();


    return 0;
}