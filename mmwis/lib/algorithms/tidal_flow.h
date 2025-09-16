#pragma once

#include <algorithm>
#include <iostream>
#include <queue>
#include <utility>
#include <vector>

#include "data_structure/flow_graph.h"
#include "data_structure/graph_access.h"
#include "definitions.h"

class tidal_flow {
   public:
    tidal_flow() = default;
    ~tidal_flow() = default;

    void init(flow_graph &fg, NodeID source, NodeID sink) {
        this->fg = &fg;
        this->source = source;
        this->sink = sink;
        high.resize(fg.number_of_nodes(), 0);
        low.resize(fg.number_of_nodes(), 0);
        preflow.resize(fg.number_of_edges(), 0);
        dist.resize(fg.number_of_nodes(), 0);
    }

    FlowType solve_max_flow_min_cut(flow_graph &fg, NodeID source,
                                    NodeID sink) {
        init(fg, source, sink);

        while (true) {
            bool sink_reachable = compute_level_graph();

            if (!sink_reachable) {
                break;
            }

            while (tidecycle() != 0) {
            }
        }

        FlowType result = 0;
        for (EdgeID e = fg.get_first_edge(source),
                    end = fg.get_first_invalid_edge(source);
             e < end; ++e) {
            result += fg.getEdgeFlow(source, e);
        }

        return result;
    }

   private:
    // Populates `edges` to an edge list of a level graph in bfs order, and
    // returns a boolean to indicate whether the sink is reachable from the source.
    bool compute_level_graph() {
        std::fill(dist.begin(), dist.end(), 0);
        dist[source] = 1;
        edges.clear();
        bool sink_reachable = false;
        queue.push(source);
        while (!queue.empty()) {
            NodeID u = queue.front();
            queue.pop();
            sink_reachable |= u == sink;
            size_t d = dist[u] + 1;
            for (EdgeID e = fg->get_first_edge(u),
                        end = fg->get_first_invalid_edge(u);
                 e < end; e++) {
                NodeID v = fg->getEdgeTarget(u, e);
                FlowType residual =
                    fg->getEdgeCapacity(u, e) - fg->getEdgeFlow(u, e);

                // ignore saturated edges
                if (residual <= 0) {
                    continue;
                }

                if (dist[v] == 0) {
                    dist[v] = d;
                    if (!sink_reachable) {
                        // only enqueue v if it wasn't seen before and the sink wasn't found yet
                        queue.push(v);
                    }
                }

                if (dist[v] == d) {
                    FlowType cap = fg->getEdgeCapacity(u, e);
                    edges.emplace_back(u, e);
                }
            }
        }

        return sink_reachable;
    }

    FlowType tidecycle() {
        const auto &level_graph = this->edges;
        size_t m = level_graph.size();
        std::fill(high.begin(), high.end(), 0);
        high[source] = std::numeric_limits<FlowType>::max();

        for (size_t i = 0; i < level_graph.size(); i++) {
            auto w = std::get<0>(level_graph[i]);
            auto e = std::get<1>(level_graph[i]);
            auto v = fg->getEdgeTarget(w, e);
            auto residual = fg->getEdgeCapacity(w, e) - fg->getEdgeFlow(w, e);

            preflow[i] = std::min(residual, high[w]);

            high[v] += preflow[i];
        }

        if (high[sink] == 0) {
            return 0;
        }

        std::fill(low.begin(), low.end(), 0);
        low[sink] = high[sink];

        for (size_t i = level_graph.size() - 1; ~i; i--) {
            auto w = std::get<0>(level_graph[i]);
            auto v = fg->getEdgeTarget(w, std::get<1>(level_graph[i]));
            preflow[i] = std::min({preflow[i], high[w] - low[w], low[v]});

            low[v] -= preflow[i];
            low[w] += preflow[i];
        }

        std::fill(high.begin(), high.end(), 0);
        high[source] = low[source];

        for (size_t i = 0; i < level_graph.size(); i++) {
            auto w = std::get<0>(level_graph[i]);
            auto e = std::get<1>(level_graph[i]);
            auto v = fg->getEdgeTarget(w, e);

            preflow[i] = std::min(preflow[i], high[w]);
            high[w] -= preflow[i];
            high[v] += preflow[i];
            fg->setEdgeFlow(w, e, fg->getEdgeFlow(w, e) + preflow[i]);
            auto e_r = fg->getReverseEdge(w, e);
            fg->setEdgeFlow(v, e_r, fg->getEdgeFlow(v, e_r) - preflow[i]);
        }

        return high[source];
    }

    flow_graph *fg{nullptr};
    NodeID source{0}, sink{0};
    std::vector<FlowType> high;
    std::vector<FlowType> low;
    std::vector<FlowType> preflow;
    std::queue<NodeID> queue;

    std::vector<std::pair<NodeID, EdgeID>> edges;
    std::vector<unsigned int> dist;
};
