/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <string>
#include "factory.h"
#include <functional>
using namespace auto_ros::common_tool;
class Base
{
public:
	Base(std::string name) : name_(name)
	{
	}
	virtual std::string Name() const { return "base" + name_; }

protected:
	std::string name_;
};

class Derived : public Base
{
public:
	Derived(std::string name) : Base(name)
	{
	}
	virtual std::string Name() const { return "derived " + name_; }
};
template <class derived>
Base *createDerived(std::string name)
{
	return new derived(name);
}
int main()
{
	Factory<std::string, Base, Base *(*)(std::string name)> factory;
	factory.Register("derived_class", createDerived<Derived>);
	std::string name = "gelinhe";
	std::shared_ptr<Base> derived_ptr = factory.CreateSharedObject("derived_class", name);
	std::cout << derived_ptr->Name() << std::endl;
}