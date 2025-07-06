#include <iostream>
#include <string>
#include <sstream>
#include "Graph.hpp"

int main() {
    Graph* graph = nullptr;
    std::string line;

    while (true) {
        std::cout << "\nEnter your choice:\n";
        if (!std::getline(std::cin, line)) break;

        std::istringstream iss(line);
        std::string command;
        iss >> command;

        if (command == "-i") {
            std::string filename;
            iss >> filename;

            delete graph;
            try {
                graph = new Graph(filename);
                std::cout << "Graph OK\n";
            } catch (...) {
                std::cerr << "Unable to open file: " << filename << "\n";
            }
        }

        else if (command == "-b") {
            unsigned long long sid;
            iss >> sid;
            if (graph) graph->bfs(sid);
        }

        else if (command == "-d") {
            unsigned long long sid;
            iss >> sid;
            if (graph) graph->dfs(sid);
        }

        else if (command == "-c") {
            if (graph) graph->compact();
        }
        else if (command == "-p") {
            unsigned long long sid, eid;
            iss >> sid >> eid;
            if (graph) graph->dijkstra(sid, eid);
        }

        else if (command == "-q") {
            delete graph;
            break;
        }

        // Ignore other commands for now
    }

    return 0;
}
