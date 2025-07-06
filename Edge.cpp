#include "Edge.hpp"

Edge::Edge(unsigned long long src, unsigned long long tgt, double dist)
    : sourceId(src), targetId(tgt), distance(dist) {}

Edge::Edge(const Edge& other)
    : sourceId(other.sourceId), targetId(other.targetId), distance(other.distance) {}

unsigned long long Edge::getSourceId() const {
    return sourceId;
}

unsigned long long Edge::getTargetId() const {
    return targetId;
}

double Edge::getDistance() const {
    return distance;
}

void Edge::setDistance(double dist) {
    distance = dist;
}
