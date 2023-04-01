#pragma once
#include "common_tool/factory.h"
#include "can_node_base.h"
#include "can_tester_node.h"
#include "can_node_qirui_eq.h"
namespace auto_ros
{
namespace can
{
template <class derived>
CanNodeBase *createNodeDerived(std::string node_yaml_file_name)
{
	return new derived(node_yaml_file_name);
}
using can_node_factory_type =
	auto_ros::common_tool::Factory<std::string, CanNodeBase, CanNodeBase *(*)(std::string node_yaml_file_name)>;
class CanNodeFactory : public can_node_factory_type
{
private:
	/* data */
public:
	CanNodeFactory()
	{
		//Register("KfLateral", createKfDerived<KfLateral>);
		Register("CanTesterNode", createNodeDerived<CanTesterNode>);   
        Register("CanNodeQiRuiEq", createNodeDerived<CanNodeQiRuiEq>);      
	}
};
} // namespace control
} // namespace auto_ros