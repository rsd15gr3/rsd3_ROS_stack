#ifndef ROUTE_H
#define ROUTE_H

#include <vector>
#include <deque>
#include <string>
#include <fstream>
#include "Node.h"

using namespace std;

class Route
{


    public:
        Route(bool load = false);
        void brickOrder(int target);
        void brickDelivery();
        void goCharge();
        void goChargeInterupt();
        void infoRoute();
        int getCurrentState();
        void setCurrentState(int state);
        bool empty();
        int  next();
        void pop();

    private:
        Nodes graph;
        std::deque<unsigned int> route;
        int currentState;

};

#endif // NODES_H
