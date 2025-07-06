#ifndef VERTEX_HPP
#define VERTEX_HPP

#include <list>
#include "Edge.hpp"

class Vertex {
private:
    unsigned long long id;
    double latitude;
    double longitude;
    std::list<Edge> edges;

public:
    Vertex(unsigned long long id, double lat, double lon);
    Vertex(const Vertex& other);

    unsigned long long getId() const;
    double getLatitude() const;
    double getLongitude() const;

    void addEdge(const Edge& edge);
    void removeEdgeTo(unsigned long long targetId);
    const std::list<Edge>& getEdges() const;
};

#endif
