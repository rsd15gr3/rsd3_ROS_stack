#include "route.h"
#include "iostream"
#include "mission/action_states.h"


Route::Route(bool load)
{
    currentState = DELIVERY;

    if(!load)
    {
        graph.Construct();
        graph.Save("graph.txt");
    }
    else
    {
        graph.Load("graph.txt");
    }

}

int Route::getCurrentState()
{
    return currentState;
}

void Route::setCurrentState(int state)
{
    currentState = state;
}

int Route::next()
{
    return route[0];
}

bool Route::empty()
{
    return route.empty();
}


void Route::brickOrder(int target)
{
     vector<unsigned int> temp;
     temp = graph.WidthSearch(currentState, BRICK);
     for(int i=0; i < temp.size(); i++)
     {
         route.push_back(temp[i]);
     }
     temp = graph.WidthSearch(BRICK, target);
     for(int i=0; i < temp.size(); i++)
     {
         route.push_back(temp[i]);
     }
     //route.push_back(CTR_IDLE);
}

void Route::brickDelivery()
{
    vector<unsigned int> temp;
    temp = graph.WidthSearch(currentState, DELIVERY);
    for(int i=0; i < temp.size(); i++)
    {
        route.push_back(temp[i]);
    }
}

void Route::goCharge()
{
    vector<unsigned int> temp;
    temp = graph.WidthSearch(currentState, CHARGE);
    for(int i=0; i < temp.size(); i++)
    {
        route.push_back(temp[i]);
    }
}

void Route::goChargeInterupt()
{
    vector<unsigned int> temp;
    //Go to charge
    temp = graph.WidthSearch(CHARGE, currentState);
    for(int i=temp.size()-1; i >= 0; i--)
    {
        route.push_front(temp[i]);
    }
    //Go to charge
    temp = graph.WidthSearch(currentState, CHARGE);
    for(int i=temp.size()-1; i >= 0; i--)
    {
        route.push_front(temp[i]);
    }
}

void Route::pop()
{
    currentState = route.front();
    route.pop_front();
}

void Route::infoRoute()
{
    for(int i=0; i< route.size(); i++)
    {
        switch (route[i])
        {
            case CTR_IDLE:
                std::cout << "Wait now" << std::endl;
            break;

            case BRICK:
                std::cout << "Get some bricks" << std::endl;
            break;

            case TRANSITION:
                std::cout << "Go to transition area" << std::endl;
            break;

            case CHARGE:
                std::cout << "Go to charge station" << std::endl;
            break;

            case DELIVERY:
                std::cout << "Deliver the bricks" << std::endl;
            break;

            case CELL_1:
                std::cout << "Go to workcell 1" << std::endl;
            break;

            case CELL_2:
                std::cout << "Go to workcell 2" << std::endl;
            break;

            case CELL_3:
                std::cout << "Go to workcell 3" << std::endl;
            break;
        }
    }
    //std::cout << std::endl;
}
