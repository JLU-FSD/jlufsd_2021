#pragma once
#include <string>
namespace auto_ros
{
namespace common_tool
{
std::string getCurrentTimeStr()
{
	time_t t = time(NULL);
	char ch[64] = {0};
	strftime(ch, sizeof(ch) - 1, "%Y-%m-%d-%H:%M:%S", localtime(&t)); //年-月-日 时-分-秒
	return ch;
}
} // namespace common_tool
} // namespace auto_ros