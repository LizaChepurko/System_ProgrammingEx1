#include "Algorithms.hpp"
#include <vector>
#include <queue>
#include <limits>
#include <unordered_set>
#include <iostream>

#define INT_MAX 2147483647 


namespace ariel{

    bool isConnected(const ariel::Graph& graph) {
    // Get the number of vertices in the graph
    int vertices = graph.vertices;

    // Create a vector to mark visited vertices
    std::vector<bool> visited(vertices, false);

    // Perform DFS from vertex 0
    DFS(graph, 0, visited);

    // Check if all vertices are visited
    for (int i = 0; i < vertices; ++i) {
        if (!visited[i]) {
            return false; // Graph is not connected
        }
    }

    return true; // Graph is connected
}

void DFS(const ariel::Graph& graph, int vertex, std::vector<bool>& visited) {
    // Mark the current vertex as visited
    visited[vertex] = true;

    // Traverse adjacent vertices
    for (int i = 0; i < graph.vertices; ++i) {
        // If there is an edge from the current vertex to vertex i and vertex i is not visited
        if (graph.adjMatrix[vertex][i] && !visited[i]) {
            // Recursively visit vertex i
            DFS(graph, i, visited);
        }
    }
}

  struct Edge {
    int source;
    int destination;
    int weight;
};

    // Function to compute the edges dynamically from the adjacency matrix
    std::vector<Edge> computeEdges(Graph g){
        std::vector<Edge> edges;
        for (int i = 0; i < g.vertices; ++i) {
            for (int j = 0; j < g.vertices; ++j) {
                if (g.adjMatrix[i][j] != 0) {
                    edges.push_back({i, j, g.adjMatrix[i][j]});
                }
            }
        }
        return edges;
    }

// Function to find the shortest path from start to end using Bellman-Ford algorithm
    int shortestPath(Graph g, int start, int end) {
        // Compute the edges dynamically
        std::vector<Edge> edges = computeEdges(g);

        // Initialize distance array with infinite values
        std::vector<int> distance(g.vertices, std::numeric_limits<int>::max());
        // Set the distance of the start vertex to 0
        distance[start] = 0;

        // Relax edges |V| - 1 times
        for (int i = 0; i < g.vertices - 1; ++i) {
            // Traverse all edges and relax them
            for (const Edge& edge : edges) {
                if (distance[edge.source] != std::numeric_limits<int>::max() &&
                    distance[edge.source] + edge.weight < distance[edge.destination]) {
                    distance[edge.destination] = distance[edge.source] + edge.weight;
                }
            }
        }

        // Check for negative cycles
        for (const Edge& edge : edges) {
            if (distance[edge.source] != std::numeric_limits<int>::max() &&
                distance[edge.source] + edge.weight < distance[edge.destination]) {
                // Negative cycle detected, return -1
                return -1;
            }
        }

        // Return the distance to the end vertex
        return distance[end];
    }

    // DFS traversal to detect cycle
    bool DFS(Graph g ,int vertex, int parent, std::vector<bool>& visited) {
        visited[vertex] = true;

        // Traverse adjacent vertices
        for (int i = 0; i < g.vertices; ++i) {
            if (g.adjMatrix[vertex][i]) {
                // If adjacent vertex is not visited, recursively call DFS
                if (!visited[i]) {
                    if (DFS(g,i, vertex, visited)) {
                        return true;
                    }
                } else if (i != parent) {
                    // If adjacent vertex is visited and not the parent, cycle found
                    return true;
                }
            }
        }

        return false;
    }

    int isContainsCycle(Graph g) {
        // Vector to keep track of visited vertices
        std::vector<bool> visited(g.vertices, false);

        // Perform DFS from each vertex
        for (int i = 0; i < g.vertices; ++i) {
            if (!visited[i] && DFS(g,i, -1, visited)) {
                return 1;
            }
        }

        // No cycle found
        return 0;
    }

        int isBipartite(Graph g) {
        // Vector to store the color of each vertex (0: uncolored, 1: color A, -1: color B)
        std::vector<int> colors(g.vertices, 0);

        // Queue for BFS traversal
        std::queue<int> q;

        // Perform BFS from each vertex
        for (int i = 0; i < g.vertices; ++i) {
            if (colors[i] == 0) {
                colors[i] = 1; // Color vertex i with color A
                q.push(i);

                while (!q.empty()) {
                    int vertex = q.front();
                    q.pop();

                    for (int j = 0; j < g.vertices; ++j) {
                        if (g.adjMatrix[vertex][j]) {
                            // If adjacent vertex j is uncolored
                            if (colors[j] == 0) {
                                colors[j] = -colors[vertex]; // Color with opposite color of vertex
                                q.push(j);
                            } else if (colors[j] == colors[vertex]) {
                                // If adjacent vertex j has same color as vertex, not bipartite
                                return 0;
                            }
                        }
                    }
                }
            }
        }

        // Build partition based on colors
        std::unordered_set<int> groupA, groupB;
        for (int i = 0; i < g.vertices; ++i) {
            if (colors[i] == 1) {
                groupA.insert(i);
            } else {
                groupB.insert(i);
            }
        }

        // Print the partition
        std::cout << "A={";
        for (int vertex : groupA) {
            std::cout << vertex << ", ";
        }
        std::cout << "}, B={";
        for (int vertex : groupB) {
            std::cout << vertex << ", ";
        }
        std::cout << "}\n";

        return 1; // Graph is bipartite
    }

     bool negativeCycle(Graph g) {
        // Initialize distance array with infinite values
        std::vector<int> distance(g.vertices, INT_MAX);
        distance[0] = 0; // Set distance of source vertex to 0

        // Relax edges V-1 times
        for (int i = 0; i < g.vertices - 1; ++i) {
            for (int u = 0; u < g.vertices; ++u) {
                for (int v = 0; v < g.vertices; ++v) {
                    if (g.adjMatrix[u][v] != 0 && distance[u] != INT_MAX && distance[u] + g.adjMatrix[u][v] < distance[v]) {
                        distance[v] = distance[u] + g.adjMatrix[u][v];
                    }
                }
            }
        }

        // Check for negative cycle
        for (int u = 0; u < g.vertices; ++u) {
            for (int v = 0; v < g.vertices; ++v) {
                if (g.adjMatrix[u][v] != 0 && distance[u] != INT_MAX && distance[u] + g.adjMatrix[u][v] < distance[v]) {
                    std::cout << "Negative cycle found!\n";
                    return true;
                }
            }
        }

        std::cout << "No negative cycle found.\n";
        return false;
    }

    

}