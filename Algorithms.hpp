
#include "Graph.hpp"
#ifndef Algorithms_HPP
#define Algorithms_HPP

namespace ariel{

    int isConnected(Graph);

    int shortestPath(Graph,int start,int end);

    int isContainsCycle(Graph);

    int isBipartite(Graph);

    bool negativeCycle(Graph);
}
#endif 