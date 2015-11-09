#ifndef NODES_H
#define NODES_H

#include <vector>
#include <string>
#include <fstream>


using namespace std;

class Nodes
{
    struct Node
    {
        vector<unsigned int> neighbours;
        unsigned int nr;
    };


    public:
        Nodes();
        void Construct();
        void Save(string file);
        void Load(string file);
        vector<unsigned int> WidthSearch(unsigned int node1, unsigned int node2); // used to find a route

    protected:
    private:
        unsigned int Get_index(unsigned int nr);
        vector<Node> node_list;
        unsigned int tree_counter;
        void Add(unsigned int nr);
        void Connect(unsigned int node1, unsigned int node2);

};

#endif // NODES_H
