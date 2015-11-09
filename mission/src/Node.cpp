#include "Node.h"
#include "iostream"
#include "queue"
#include "mission/action_states.h"

struct search_graph
{
    unsigned int node;
    vector<unsigned int> path;
};

void Nodes::Construct()
{
    Add(CHARGE);
    Add(BRICK);
    Add(DELIVERY);
    Add(TRANSITION);
    Add(CELL_1);
    Add(CELL_2);
    Add(CELL_3);

    Connect(CELL_1,TRANSITION);
    Connect(CELL_2,TRANSITION);
    Connect(CELL_3,TRANSITION);

    Connect(CHARGE,TRANSITION);
    Connect(BRICK,TRANSITION);
    Connect(DELIVERY,TRANSITION);

    Connect(CHARGE,BRICK);
    Connect(CHARGE,DELIVERY);
    Connect(BRICK,DELIVERY);
}

Nodes::Nodes()
{
    tree_counter = 0;
}

void Nodes::Save(string file)
{
    ofstream save( file.c_str() );

    save << node_list.size() << endl;
    for(unsigned int i = 0; i< node_list.size(); i++)
    {
        save << node_list[i].neighbours.size() << " ";
        for(unsigned int neighbour = 0; neighbour < node_list[i].neighbours.size(); neighbour++)
        {
            save << node_list[i].neighbours[neighbour] << " ";
        }
        save << endl;
        save << node_list[i].nr << endl;
    }
    save.close();
}

void Nodes::Load(string file)
{
    ifstream load( file.c_str() );

    int node_count;
    load >> node_count;
    for(int i = 0; i< node_count; i++)
    {
        Node temp;
        int neighbour_count;
        load >> neighbour_count;
        for(int neighbour = 0; neighbour < neighbour_count; neighbour++)
        {
            unsigned int neighbour_temp;
            load >> neighbour_temp;
            temp.neighbours.push_back(neighbour_temp);
        }
        load >> temp.nr;

        node_list.push_back(temp);
    }
    load.close();
}

/*
Adds a note that can´t be connected yet, and gives it a new tree number
*/
void Nodes::Add(unsigned int nr)
{
    Node temp;
    tree_counter++;
    temp.nr = nr;
    temp.neighbours.empty();
    node_list.push_back(temp);
}

/*
Takes in the number of two nodes, checks if they have the same tree number and if not makes collision check and connect if it´s collision free
*/
void Nodes::Connect(unsigned int node1, unsigned int node2)
{
    node_list[Get_index(node1)].neighbours.push_back(node2);
    node_list[Get_index(node2)].neighbours.push_back(node1);
}

//makes connection between node nr and the node list index
unsigned int Nodes::Get_index(unsigned int nr)
{
    for(unsigned int i = 0; i<node_list.size(); i++)
    {
        if(nr == node_list[i].nr)
        {
            return i;
        }
    }

    throw("not existing node");
}

vector<unsigned int> Nodes::WidthSearch(unsigned int node1, unsigned int node2)
{
    search_graph current;
    current.node = node1;
    //current.path.push_back(node1);

   queue<search_graph> options;

   while (current.node != node2)
   {
      for (unsigned int i = 0; i < node_list[ Get_index(current.node) ].neighbours.size(); i++)
      {
          search_graph temp;
          temp.node = node_list[ Get_index(current.node) ].neighbours[i];
          temp.path = current.path;
          temp.path.push_back(temp.node);
          options.push(temp);
      }

      current = options.front();
      options.pop();
   }

   return current.path;
}

