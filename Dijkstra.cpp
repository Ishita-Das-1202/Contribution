#include <iostream>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

#define INF INT_MAX

// Data structure to store a graph edge
struct Edge {
    int src, dest, weight;
};

// Data structure to store a heap node
struct Node {
    int vertex, weight;
};

// Comparator for the priority queue
struct compare {
    bool operator()(const Node& l, const Node& r) {
        return l.weight > r.weight;
    }
};

// Function to add an edge to the graph
void addEdge(vector<Edge> edges[], int src, int dest, int weight) {
    Edge edge = {src, dest, weight};
    edges[src].push_back(edge);
}

// Function to run Dijkstra's algorithm
void dijkstra(vector<Edge> edges[], int src, int V) {
    priority_queue<Node, vector<Node>, compare> minHeap;
    vector<int> dist(V, INF);
    minHeap.push({src, 0});
    dist[src] = 0;

    while (!minHeap.empty()) {
        Node node = minHeap.top();
        minHeap.pop();
        int u = node.vertex;

        for (const auto& edge : edges[u]) {
            int v = edge.dest;
            int weight = edge.weight;

            if (dist[u] != INF && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                minHeap.push({v, dist[v]});
            }
        }
    }

    // Print the shortest distances to all the vertices
    cout << "Vertex Distance from Source" << endl;
    for (int i = 0; i < V; i++) {
        cout << i << "\t\t" << dist[i] << endl;
    }
}

// Dijkstra's Algorithm in C++
int main() {
    // Initialize a vector of edges
    const int V = 9;
    vector<Edge> edges[V];

    // Add edges to the graph
    addEdge(edges, 0, 1, 4);
    addEdge(edges, 0, 7, 8);
    addEdge(edges, 1, 2, 8);
    addEdge(edges, 1, 7, 11);
    addEdge(edges, 2, 3, 7);
    addEdge(edges, 2, 8, 2);
    addEdge(edges, 2, 5, 4);
    addEdge(edges, 3, 4, 9);
    addEdge(edges, 3, 5, 14);
    addEdge(edges, 4, 5, 10);
    addEdge(edges, 5, 6, 2);
    addEdge(edges, 6, 7, 1);
    addEdge(edges, 6, 8, 6);
    addEdge(edges, 7, 8, 7);

    // Run Dijkstra's algorithm from vertex 0
    dijkstra(edges, 0, V);

    return 0;
}
