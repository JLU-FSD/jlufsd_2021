#pragma once
#include <iostream>
class watchDog
{
public:
	watchDog(int foodPerTurn = 0, int hungerPerTurn = 0, int hungerMin = 0,
			 int criticalPoint = 50, int hungerMax = 100);
	void setAllParameter(int _foodPerTurn = 0, int _hungerPerTurn = 0, int _hungerMin = 0,
						 int _criticalPoint = 50, int _hungerMax = 100);
	void giveFood();
	void noFood();
	bool isDied();
	/**get function family,shoud not pay much attention********/
	int getHunger();
	int getFoodPerTurn();
	int getCriticalPoint();
	int getHungerMin();
	int getHungerMax();
	/**set function family,shoud not pay much attention********/
	void setHunger(double _hunger);
	void setFoodPerTurn(double _foodPerTurn);
	void setCriticalPoint(double _criticalPoint);
	void setHungerMin(double hungerMin);
	void setHungerMax(double hungerMax);
	/**private member******************************************/
private:
	int hunger;		   //饥饿度
	int foodPerTurn;   //如果喂狗每次喂的食物
	int hungerPerTurn; //如果没来得及喂狗，狗饥饿度的增长度数
	int criticalPoint; //饿得不行的点，介于max和min之间
	int hungerMin;
	int hungerMax;
};