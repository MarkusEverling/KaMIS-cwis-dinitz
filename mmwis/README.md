# Code changes
- Tidal Flow is implemented in `lib/algorithms/tidal_flow.h`
- Dinitz's Algorithm is implemented in `lib/algorithms/dinitz.h`
- The CSR flow graph is implemented in `lib/data_structure/dinitz_flow_graph.h`. The new pruning construction of the flow graph is done in its constructor.
- The CWIS reduction implementation is located in `lib/mis/kernel/reductions.cpp`. There we change the flow graph construction, the flow algorithm used, and we make the graph scan a full BFS.

To build the `weighted_reduce` executable using our implementation of the CWIS reduction:

```sh
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j weighted_reduce
```

# Timings

The timings for the CWIS reduction in isolation for all input instances are listed in `cwis.csv`. They include runtimes for the baseline Push-Relabel, our Tidal Flow and Dinitz, and Boost Push-Relabel, all given in microseconds. For Dinitz's Algorithm, we also test two versions: `DAIterative` uses our new flow graph, but uses iterative DFS for its `augment()` implementation instead of recursive DFS. `DAOldFG` uses recursive DFS, but uses the existing adjacency list based `flow_graph` data structure instead of our new CSR based flow graph. Since both are consistently significantly slower than the version using the recursive DFS and the CSR based flow graph, we use that version as our final implementation.

The timings for the full reduction vector (i.e. for a `weighted_reduce` invocation) are listed in `weighted_reduce.csv`. They include reduction offsets for the baseline, for a version changing only the flow algorithm to Dinitz's Algorithm but leaving everything else the same, as well as for our final version, doing a full BFS on the residual graph. Note that the reduction offsets are identical in almost all cases, and do not show a clear winner for the remaining cases. The times given are the reduction times reported by the `weighted_reduce` executable, given in seconds. `PushRelabel` represents the baseline, `Dinitz` represents a version changing only the flow algorithm, but keeping the old graph scan and flow graph data structure, as well as the old unpruned graph construction, `FullBFS` is a version using Dinitz's Algorithm and doing a full BFS on the residual grpah, and `NewFG` is our final version, including the CSR based flow graph and the flow graph pruning. As `NewFG` is consistently the fastest among the Dinitz based approaches, it is the version we use for our final results.