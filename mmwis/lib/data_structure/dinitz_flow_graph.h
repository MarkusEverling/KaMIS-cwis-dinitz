#pragma once

#include "KaHIP/lib/definitions.h"
#include "branch_and_reduce_algorithm.h"

// using namespace mmwis;

class dinitz_flow_graph {
    NodeID num_nodes;
    EdgeID num_edges;

    // For edge u->v with index e, heads[e] == v.
    std::vector<NodeID> heads;
    std::vector<FlowType> flows, capacities;
    // For edge u->v with index e, rev_idx[e] is the index of edge v->u.
    std::vector<EdgeID> rev_idx;
    // For node u, its outgoing edges are stored in the range
    // [edge_ptr[u],edge_ptr[u+1])
    std::vector<EdgeID> edge_ptr;

  public:
    dinitz_flow_graph(
        mmwis::branch_and_reduce_algorithm::graph_status *status) {
        num_nodes = 2 * status->n + 2;

        edge_ptr.resize(num_nodes + 1);

        NodeID source = num_nodes - 2, sink = num_nodes - 1;

        // We first do one counting pass, counting the number of edges
        // (including reverse) per node. For node u, we store this count
        // in edge_ptr[u + 1].
        for (NodeID node = 0; node < status->n; node++) {
            if (status->node_status[node] ==
                mmwis::branch_and_reduce_algorithm::IS_status::not_set) {
                // One edge from the source to the node...
                edge_ptr[source + 1]++;
                // ... and one back to the source.
                edge_ptr[node + 1]++;

                // Same with edges to and from the sink.
                edge_ptr[node + status->n + 1]++;
                edge_ptr[sink + 1]++;

                for (NodeID neighbor : status->graph[node]) {
                    if (status->node_status[node] ==
                        mmwis::branch_and_reduce_algorithm::IS_status::
                            not_set) {

                        // Account for forward and backward edges between the
                        // layers
                        edge_ptr[node + 1]++;
                        edge_ptr[neighbor + status->n + 1]++;
                    }
                }
            }
        }

        // Now we compute the prefix sum of edge_ptr, to get the final segments
        // for each node.
        for (NodeID i = 1; i < num_nodes + 1; i++) {
            edge_ptr[i] += edge_ptr[i - 1];
        }

        num_edges = edge_ptr[num_nodes];
        heads.resize(num_edges);
        flows.resize(num_edges);
        capacities.resize(num_edges);
        rev_idx.resize(num_edges);

        // For each node, keep track of how many edges
        // in its segment have already been filled in.
        std::vector<EdgeID> progress = edge_ptr;

        // Adds an edge u->v with capacity cap and an edge v->u with capacity 0.
        auto add_edges = [&](NodeID u, NodeID v, FlowType cap) {
            EdgeID e = progress[u]++, e_r = progress[v]++;

            heads[e] = v;
            heads[e_r] = u;
            capacities[e] = cap;
            rev_idx[e] = e_r;
            rev_idx[e_r] = e;
        };

        // Now we do the second pass over the input graph, to fill in all the
        // edges.
        for (NodeID node = 0; node < status->n; node++) {
            if (status->node_status[node] ==
                mmwis::branch_and_reduce_algorithm::IS_status::not_set) {
                add_edges(source, node, status->weights[node]);
                add_edges(node + status->n, sink, status->weights[node]);

                for (NodeID neighbor : status->graph[node]) {
                    if (status->node_status[node] ==
                        mmwis::branch_and_reduce_algorithm::IS_status::
                            not_set) {
                        add_edges(node, neighbor + status->n,
                                  status->weights[node]);
                    }
                }
            }
        }
    }

    NodeID number_of_nodes() const { return num_nodes; }
    EdgeID number_of_edges() const { return num_edges; }

    // We mirror the API of flow_graph (except with more const correctness)
    // to minimize the changes in the caller code. Inlining should get rid
    // of the overhead of unnecessary arguments anyways.
    NodeID getEdgeTarget([[maybe_unused]] NodeID source, EdgeID e) const {
        return heads[e];
    }

    FlowType getEdgeCapacity([[maybe_unused]] NodeID source, EdgeID e) const {
        return capacities[e];
    }

    FlowType getEdgeFlow([[maybe_unused]] NodeID source, EdgeID e) const {
        return flows[e];
    }

    void setEdgeFlow([[maybe_unused]] NodeID source, EdgeID e, FlowType f) {
        flows[e] = f;
    }

    EdgeID getReverseEdge([[maybe_unused]] NodeID source, EdgeID e) const {
        return rev_idx[e];
    }

    EdgeID get_first_edge(NodeID node) const { return edge_ptr[node]; }
    EdgeID get_first_invalid_edge(NodeID node) const {
        return edge_ptr[node + 1];
    }
};
