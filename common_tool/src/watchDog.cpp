#include "common_tool/watchDog.h"
watchDog::watchDog(int _foodPerTurn, int _hungerPerTurn, int _hungerMin,
				   int _criticalPoint, int _hungerMax) : foodPerTurn(_foodPerTurn), hungerPerTurn(_hungerPerTurn),
														 hungerMin(_hungerMin), criticalPoint(_criticalPoint), hungerMax(_hungerMax)
{

	hunger = 0; //为零表示一点不饿
};
void watchDog::giveFood()
{
	if (hunger >= hungerMin)
	{
		hunger -= foodPerTurn;
	}
}
void watchDog::noFood()
{

	if (hunger <= hungerMax)
	{
		hunger += hungerPerTurn;
	}
}
bool watchDog::isDied()
{

	if (hunger > criticalPoint)
		return true;
	else
		return false;
}
/*start*get function family,shoud not pay much attention********/
int watchDog::getHunger()
{
	return hunger;
}
int watchDog::getFoodPerTurn()
{
	return foodPerTurn;
}
int watchDog::getCriticalPoint()
{
	return criticalPoint;
}
int watchDog::getHungerMin()
{
	return hungerMin;
}
int watchDog::getHungerMax()
{
	return hungerMax;
}
/*end*get function family,shoud not pay much attention********/

/*start*set function family,shoud not pay much attention********/
void watchDog::setHunger(double _hunger)
{
	hunger = _hunger;
}
void watchDog::setFoodPerTurn(double _foodPerTurn)
{
	foodPerTurn = _foodPerTurn;
}

void watchDog::setCriticalPoint(double _criticalPoint)
{
	criticalPoint = _criticalPoint;
}
void watchDog::setHungerMin(double _hungerMin)
{
	hungerMin = _hungerMin;
}
void watchDog::setHungerMax(double _hungerMax)
{
	hungerMax = _hungerMax;
}
void watchDog::setAllParameter(int _foodPerTurn, int _hungerPerTurn, int _hungerMin,
							   int _criticalPoint, int _hungerMax)
{
	std::cout << "##############" << _foodPerTurn << std::endl;
	foodPerTurn = _foodPerTurn;
	hungerPerTurn = _hungerPerTurn;
	hungerMin = _hungerMin;
	criticalPoint = _criticalPoint;
	hungerMax = _hungerMax;
}

/*end*set function family,shoud not pay much attention********/
