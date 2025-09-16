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

