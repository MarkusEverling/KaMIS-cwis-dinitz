#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "data_structure/dinitz_flow_graph.h"

class dinitz {
  public:
    void init(dinitz_flow_graph &fg) {
        this->fg = &fg;
        n = fg.number_of_nodes();
        this->source = n - 2;
        this->sink = n - 1;
        queue.resize(n);
        ranks.resize(n);
        next.resize(n);
    }

    FlowType solve_max_flow_min_cut(dinitz_flow_graph &fg,
                                    [[maybe_unused]] NodeID source,
                                    [[maybe_unused]] NodeID sink) {
        init(fg);
        return run_alg();
    }

  private:
    FlowType run_alg() {
        FlowType result = 0;
        while (compute_ranks()) {
            for (NodeID i = 0; i < fg->number_of_nodes(); i++) {
                next[i] = fg->get_first_edge(i);
            }

            FlowType aug;
            while ((aug = augment(source,
                                  std::numeric_limits<FlowType>::max())) != 0) {
                result += aug;
            }
        }

        return result;
    }

    bool compute_ranks() {
        std::fill(ranks.begin(), ranks.end(), 0);

        size_t qstart = 0, qend = 1;
        queue[0] = source;
        ranks[source] = 1;

        while (qstart != qend) {
            NodeID x = queue[qstart++];
            if (x == sink)
                return true;

            for (EdgeID e = fg->get_first_edge(x),
                        end = fg->get_first_invalid_edge(x);
                 e != end; e++) {
                NodeID y = fg->getEdgeTarget(x, e);
                FlowType residual =
                    fg->getEdgeCapacity(x, e) - fg->getEdgeFlow(x, e);
                if (residual > 0 && ranks[y] == 0) {
                    ranks[y] = ranks[x] + 1;
                    queue[qend++] = y;
                }
            }
        }

        return false;
    }

    FlowType augment(NodeID x, FlowType flow) {
        if (x == sink)
            return flow;
        EdgeID end = fg->get_first_invalid_edge(x);

        while (next[x] != end) {
            EdgeID e = next[x];
            NodeID y = fg->getEdgeTarget(x, e);
            FlowType residual =
                fg->getEdgeCapacity(x, e) - fg->getEdgeFlow(x, e);
            if (residual > 0 && ranks[y] == ranks[x] + 1) {
                FlowType eps = augment(y, std::min(residual, flow));
                if (eps > 0) {
                    fg->setEdgeFlow(x, e, fg->getEdgeFlow(x, e) + eps);
                    EdgeID e_r = fg->getReverseEdge(x, e);
                    fg->setEdgeFlow(y, e_r, fg->getEdgeFlow(y, e_r) - eps);
                    return eps;
                }
            }

            next[x]++;
        }

        return 0;
    }

    dinitz_flow_graph *fg{nullptr};
    NodeID source{0}, sink{0}, n{0};

    std::vector<NodeID> queue;
    std::vector<int> ranks;
    std::vector<EdgeID> next;
};