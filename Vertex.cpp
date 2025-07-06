#include "Vertex.hpp"

Vertex::Vertex(unsigned long long id, double lat, double lon)
    : id(id), latitude(lat), longitude(lon) {}

Vertex::Vertex(const Vertex& other)
    : id(other.id), latitude(other.latitude), longitude(other.longitude), edges(other.edges) {}

unsigned long long Vertex::getId() const {
    return id;
}

double Vertex::getLatitude() const {
    return latitude;
}

double Vertex::getLongitude() const {
    return longitude;
}

void Vertex::addEdge(const Edge& edge) {
    edges.push_back(edge);
}

void Vertex::removeEdgeTo(unsigned long long targetId) {
    edges.remove_if([targetId](const Edge& e) {
        return e.getTargetId() == targetId;
    });
}

const std::list<Edge>& Vertex::getEdges() const {
    return edges;
}