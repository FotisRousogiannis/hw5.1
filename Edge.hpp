#ifndef EDGE_HPP
#define EDGE_HPP

class Edge {
private:
    unsigned long long sourceId;
    unsigned long long targetId;
    double distance;

public:
    Edge(unsigned long long src, unsigned long long tgt, double dist);
    Edge(const Edge& other);

    unsigned long long getSourceId() const;
    unsigned long long getTargetId() const;
    double getDistance() const;

    void setDistance(double dist);
};

#endif
