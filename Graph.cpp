#include "Graph.hpp"
#include <iostream>
#include "tinyxml2.h"
#include <cmath>
#include <set>
#include <algorithm> // for std::isspace
#include <queue>
#include <stack>
#include <unordered_set>
#include <iomanip>
using namespace tinyxml2;

const double EARTH_RADIUS = 6378137.0;
const double PI = 3.141592653589793;

static double deg2rad(double deg) {
    return deg * (PI / 180.0);
}

static double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dLat = deg2rad(lat2 - lat1);
    double dLon = deg2rad(lon2 - lon1);
    lat1 = deg2rad(lat1);
    lat2 = deg2rad(lat2);
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS * c;
}

static double getFactor(const std::string& type) {
    if (type == "motorway" || type == "trunk") return 0.5;
    if (type == "primary" || type == "secondary") return 0.75;
    if (type == "tertiary" || type == "residential") return 1.0;
    if (type == "living_street" || type == "unclassified") return 1.25;
    if (type == "service" || type == "track") return 1.5;
    return -1.0;
}

static std::string clean(const char* s) {
    std::string str = s ? s : "";
    str.erase(std::remove_if(str.begin(), str.end(), [](unsigned char c) {
        return std::isspace(c) || c == static_cast<unsigned char>(0xA0);
    }), str.end());
    return str;
}

Graph::Graph(const std::string& osmFilePath) {
    XMLDocument doc;
    XMLError result = doc.LoadFile(osmFilePath.c_str());

    if (result == XML_ERROR_FILE_NOT_FOUND) {
        std::cout << "Unable to open file: " << osmFilePath << "\n\n";
        return;
    } else if (result != XML_SUCCESS) {
        std::cout << "Invalid format for file: " << osmFilePath << "\n\n";
        return;
    }

    std::unordered_map<unsigned long long, std::pair<double, double>> nodeMap;
    XMLElement* root = doc.FirstChildElement("osm");
    if (!root) return;

    for (XMLElement* node = root->FirstChildElement("node"); node; node = node->NextSiblingElement("node")) {
        const char* idAttr = node->Attribute("id");
        const char* latAttr = node->Attribute("lat");
        const char* lonAttr = node->Attribute("lon");

        if (!idAttr || !latAttr || !lonAttr) continue;

        try {
            unsigned long long id = std::stoull(clean(idAttr));
            double lat = std::stod(clean(latAttr));
            double lon = std::stod(clean(lonAttr));

            nodeMap[id] = {lat, lon};
            addVertex(id, lat, lon);
        } catch (...) {
            continue;
        }
    }

    for (XMLElement* way = root->FirstChildElement("way"); way; way = way->NextSiblingElement("way")) {
        bool valid = false;
        std::string type = "";
        bool oneway = false;

        for (XMLElement* tag = way->FirstChildElement("tag"); tag; tag = tag->NextSiblingElement("tag")) {
            const char* kAttr = tag->Attribute("k");
            const char* vAttr = tag->Attribute("v");
            if (!kAttr || !vAttr) continue;
            std::string k = kAttr;
            std::string v = vAttr;
            if (k == "highway") {
                type = v;
                valid = getFactor(type) != -1.0;
            } else if (k == "oneway") {
                oneway = (v == "yes" || v == "1");
            }
        }
        if (!valid) continue;
        double factor = getFactor(type);

        std::vector<unsigned long long> refs;
        for (XMLElement* nd = way->FirstChildElement("nd"); nd; nd = nd->NextSiblingElement("nd")) {
            const char* refAttr = nd->Attribute("ref");
            if (!refAttr) continue;

            std::string cleaned = clean(refAttr);
            try {
                unsigned long long refId = std::stoull(cleaned);
                refs.push_back(refId);
            } catch (...) {
                continue;
            }
        }

        for (size_t i = 0; i + 1 < refs.size(); ++i) {
            unsigned long long from = refs[i];
            unsigned long long to = refs[i + 1];

            if (!vertexExists(from) || !vertexExists(to)) continue;

            double dist = haversine(
                getVertex(from).getLatitude(), getVertex(from).getLongitude(),
                getVertex(to).getLatitude(), getVertex(to).getLongitude()
            ) * factor;

            getVertex(from).addEdge(Edge(from, to, dist));
            if (!oneway) {
                getVertex(to).addEdge(Edge(to, from, dist));
            }
        }
    }

    std::set<unsigned long long> connected;
    for (const auto& v : vertices) {
        if (!v.getEdges().empty()) {
            connected.insert(v.getId());
            for (const auto& e : v.getEdges()) {
                connected.insert(e.getTargetId());
            }
        }
    }
    std::vector<unsigned long long> toRemove;
    for (const auto& [id, idx] : vertexIndex) {
        if (!connected.count(id)) {
            toRemove.push_back(id);
        }
    }
    for (unsigned long long id : toRemove) {
        removeVertex(id);
    }

    std::cout << "Graph OK\n";
}

void Graph::addVertex(unsigned long long id, double lat, double lon) {
    if (vertexIndex.count(id)) return;
    vertices.emplace_back(id, lat, lon);
    vertexIndex[id] = vertices.size() - 1;
}

void Graph::removeVertex(unsigned long long id) {
    if (!vertexIndex.count(id)) return;
    unsigned int index = vertexIndex[id];

    for (auto& vertex : vertices) {
        vertex.removeEdgeTo(id);
    }

    if (index != vertices.size() - 1) {
        std::swap(vertices[index], vertices.back());
        vertexIndex[vertices[index].getId()] = index;
    }

    vertexIndex.erase(id);
    vertices.pop_back();
}

bool Graph::vertexExists(unsigned long long id) const {
    return vertexIndex.count(id) > 0;
}

Vertex& Graph::getVertex(unsigned long long id) {
    return vertices[vertexIndex.at(id)];
}

const Vertex& Graph::getVertex(unsigned long long id) const {
    return vertices.at(vertexIndex.at(id));
}

void Graph::bfs(unsigned long long startId) {
    if (!vertexExists(startId)) {
        std::cout << "Invalid node ID\n";
        return;
    }

    std::queue<unsigned long long> q;
    std::unordered_set<unsigned long long> visited;

    q.push(startId);
    visited.insert(startId);

    while (!q.empty()) {
        unsigned long long currentId = q.front();
        q.pop();

        std::cout << currentId << "\n";

        const Vertex& current = getVertex(currentId);
        for (const Edge& edge : current.getEdges()) {
            unsigned long long neighbor = edge.getTargetId();
            if (!visited.count(neighbor)) {
                visited.insert(neighbor);
                q.push(neighbor);
            }
        }
    }
}

void Graph::dfs(unsigned long long startId) {
    if (!vertexExists(startId)) {
        std::cout << "Invalid node ID\n";
        return;
    }

    std::stack<unsigned long long> s;
    std::unordered_set<unsigned long long> visited;

    s.push(startId);

    while (!s.empty()) {
        unsigned long long currentId = s.top();
        s.pop();

        if (visited.count(currentId)) continue;

        visited.insert(currentId);
        std::cout << currentId << "\n";

        const Vertex& current = getVertex(currentId);
        for (const Edge& edge : current.getEdges()) {
            unsigned long long neighbor = edge.getTargetId();
            if (!visited.count(neighbor)) {
                s.push(neighbor);
            }
        }
    }
}

void Graph::compact() {
    std::unordered_set<unsigned long long> toRemove;

    for (const auto& [id, idx] : vertexIndex) {
        Vertex& v = vertices[idx];
        const auto& edges = v.getEdges();

        std::vector<Edge> inEdges;
        std::vector<Edge> outEdges(edges.begin(), edges.end());

        for (const auto& [otherId, otherIdx] : vertexIndex) {
            if (otherId == id) continue;
            for (const Edge& e : vertices[otherIdx].getEdges()) {
                if (e.getTargetId() == id) inEdges.push_back(e);
            }
        }

        if (inEdges.size() == 1 && outEdges.size() == 1) {
            const Edge& in = inEdges[0];
            const Edge& out = outEdges[0];
            if (in.getSourceId() != out.getTargetId()) {
                unsigned long long newFrom = in.getSourceId();
                unsigned long long newTo = out.getTargetId();

                double dist = haversine(
                    getVertex(newFrom).getLatitude(), getVertex(newFrom).getLongitude(),
                    getVertex(newTo).getLatitude(), getVertex(newTo).getLongitude()
                );

                double factor = 1.0;
                getVertex(newFrom).addEdge(Edge(newFrom, newTo, dist * factor));
                toRemove.insert(id);
            }
        } else if (inEdges.size() == 2 && outEdges.size() == 2) {
            unsigned long long src1 = inEdges[0].getSourceId();
            unsigned long long src2 = inEdges[1].getSourceId();
            unsigned long long tgt1 = outEdges[0].getTargetId();
            unsigned long long tgt2 = outEdges[1].getTargetId();

            if ((src1 == tgt1 && src2 == tgt2) || (src1 == tgt2 && src2 == tgt1)) {
                Vertex& v1 = getVertex(src1);
                Vertex& v2 = getVertex(src2);

                double d = haversine(
                    v1.getLatitude(), v1.getLongitude(),
                    v2.getLatitude(), v2.getLongitude()
                );

                double factor = 1.0;
                v1.addEdge(Edge(src1, src2, d * factor));
                v2.addEdge(Edge(src2, src1, d * factor));
                toRemove.insert(id);
            }
        }
    }

    for (unsigned long long id : toRemove) {
        removeVertex(id);
    }

    std::cout << "Compact OK\n";
}
void Graph::dijkstra(unsigned long long sid, unsigned long long eid) {
    if (!vertexExists(sid) || !vertexExists(eid)) {
        std::cout << "Invalid start or end node\n";
        return;
    }

    std::unordered_map<unsigned long long, double> dist;
    std::unordered_map<unsigned long long, unsigned long long> prev;
    std::unordered_set<unsigned long long> visited;

    using Pair = std::pair<double, unsigned long long>;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<>> pq;

    for (const auto& [id, idx] : vertexIndex)
        dist[id] = std::numeric_limits<double>::infinity();
    dist[sid] = 0;
    pq.emplace(0.0, sid);

    while (!pq.empty()) {
        auto [curDist, u] = pq.top(); pq.pop();
        if (visited.count(u)) continue;
        visited.insert(u);

        for (const Edge& e : getVertex(u).getEdges()) {
            unsigned long long v = e.getTargetId();
            double alt = curDist + e.getDistance();
            if (alt < dist[v]) {
                dist[v] = alt;
                prev[v] = u;
                pq.emplace(alt, v);
            }
        }
    }

    if (dist[eid] == std::numeric_limits<double>::infinity()) {
        std::cout << "No path found\n";
        return;
    }

    std::vector<unsigned long long> path;
    for (unsigned long long at = eid; at != sid; at = prev[at])
        path.push_back(at);
    path.push_back(sid);
    std::reverse(path.begin(), path.end());

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        double d = haversine(
            getVertex(path[i]).getLatitude(), getVertex(path[i]).getLongitude(),
            getVertex(path[i + 1]).getLatitude(), getVertex(path[i + 1]).getLongitude()
        );
        std::cout << "[" << path[i] << " -> " << path[i + 1] << "] " << std::fixed << std::setprecision(3) << d << "\n";
    }

    std::cout << "\n\nhttps://www.google.com/maps/dir/";
    for (size_t i = 0; i < path.size(); ++i) {
        const Vertex& v = getVertex(path[i]);
        std::cout << v.getLatitude() << "," << v.getLongitude();
        if (i + 1 < path.size()) std::cout << "/";
    }
    std::cout << "\n";
}
