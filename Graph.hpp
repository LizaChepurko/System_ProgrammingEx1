
#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>

namespace ariel {
    class Graph {
    public:
        std::vector<std::vector<int>> adjMatrix;
        int vertices;

        void loadGraph(const std::vector<std::vector<int>>& graph);
        void printGraph();
    };
}

#endif // GRAPH_HPP
