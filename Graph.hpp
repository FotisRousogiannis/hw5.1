#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <vector>
#include <unordered_map>
#include <string>
#include "Vertex.hpp"

class Graph {
private:
    std::vector<Vertex> vertices;
    std::unordered_map<unsigned long long, unsigned int> vertexIndex;

public:
    Graph(const std::string& osmFilePath);
    void addVertex(unsigned long long id, double lat, double lon);
    void removeVertex(unsigned long long id);

    // Άλλες συναρτήσεις που θα προστεθούν αργότερα:
    // - Dijkstra
    void dijkstra(unsigned long long startId, unsigned long long endId);
    // - Compact
    void compact();
    // - BFS
    void bfs(unsigned long long startId);
    // - DFS
    void dfs(unsigned long long startId);
    // Προαιρετικά βοηθητικά
    bool vertexExists(unsigned long long id) const;
    Vertex& getVertex(unsigned long long id);
    const Vertex& getVertex(unsigned long long id) const;
};

#endif