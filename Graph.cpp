#include "Graph.hpp"
#include <iostream>
#include <stdexcept>

namespace ariel {

    void Graph::loadGraph(const std::vector<std::vector<int>>& graph) {
        int n = graph.size();
        if (n == 0 || n != static_cast<int>(graph[0].size())) {
            throw std::invalid_argument("Invalid graph representation");
        }

        vertices = n;
        adjMatrix = graph;
    }

    void Graph::printGraph() {
        for (int i = 0; i < vertices; i++) {
            std::cout << i << " : ";
            for (int j = 0; j < vertices; j++)
                std::cout << adjMatrix[i][j] << " ";
            std::cout << "\n";
        }
    }

} // namespace ariel




// int main() {
//   ariel::Graph g;
//   vector<vector<int>> graph = {
//         {0, 1, 0},
//         {1, 0, 1},
//         {0, 1, 0}
//   };
//   g.loadGraph(graph);

//   g.printGraph();
// }
