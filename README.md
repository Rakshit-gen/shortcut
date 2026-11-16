# shortcut

A road routing engine in Go, built on OpenStreetMap data for South India. It starts
with plain Dijkstra and works up to Contraction Hierarchies. Every faster
algorithm is checked against Dijkstra on the same random queries, and the two
have to agree exactly.

## Quickstart

```
go build -o bin/shortcut ./cmd/shortcut
curl -LO https://download.geofabrik.de/asia/india/southern-zone-latest.osm.pbf
bin/shortcut import southern-zone-latest.osm.pbf south.graph
bin/shortcut landmarks south.graph        # for -algo alt
bin/shortcut contract south.graph         # for -algo ch
bin/shortcut route south.graph 12.9716,77.5946 12.2958,76.6394 -algo ch   # Bengaluru to Mysuru
bin/shortcut bench south.graph -algo ch -n 1000 -check
```

## Measured vs Claimed

Every number below states what it measures and how. Update this table
instead of writing new numbers straight into prose elsewhere (portfolio site,
resume). Keep one source of truth and link to it.

| Claim | Value | How measured | Date |
|---|---|---|---|
| Graph size | 26,863,660 nodes, 55,854,321 edges (largest SCC, 99.4% of imported nodes) | `shortcut import`, Geofabrik southern-zone extract (534 MB), Apple M4, 16 GB | 2026-11-17 |
| Import time / peak memory | 57 s / 3.7 GB RSS | `/usr/bin/time -l shortcut import`, same machine | 2026-11-17 |
| Landmark preprocessing (8 landmarks) | 1 min 46 s, 1.6 GB file | `shortcut landmarks -k 8` | 2026-11-17 |
| CH preprocessing | 4 min 57 s, 6.4 GB peak RSS, 53.5M up + 53.5M down edges, 1.4 GB file | `/usr/bin/time -l shortcut contract`, nothing else running. Peak RSS was 3.9 GB in an earlier run, so it depends on GC timing | 2026-11-17 |
| Dijkstra p50 / p99 | 2.56 s / 5.10 s, 13.3M settled avg | `shortcut bench -algo dijkstra -n 100 -seed 1`, random node pairs, nothing else running | 2026-11-17 |
| Bidirectional Dijkstra p50 / p99 | 1.83 s / 5.31 s, 10.6M settled avg | same pairs, `-algo bidi` | 2026-11-17 |
| A* p50 / p99 | 1.59 s / 9.02 s, 7.0M settled avg | same pairs, `-algo astar`. Settles about half as many nodes as Dijkstra, but a haversine per node makes long queries slower | 2026-11-17 |
| ALT p50 / p99 | 134 ms / 1.38 s, 1.35M settled avg | same pairs, `-algo alt` | 2026-11-17 |
| CH p50 / p99 | 0.98 ms / 4.13 ms, 668 settled avg, path unpacked | `-algo ch -n 10000 -seed 1`. About 2,600x faster than Dijkstra at the median | 2026-11-17 |
| Correctness on the real graph | 0 of 100 differ from Dijkstra for each of bidi, A*, ALT and CH | `shortcut bench -algo X -n 100 -seed 1 -check` | 2026-11-17 |
| Bengaluru to Mysuru (one query) | 1h32m55s free flow, 142.7 km. Settled: Dijkstra 2.08M (396 ms), bidi 1.27M (278 ms), A* 400K (131 ms), ALT 28K (49 ms), CH 383 (3.7 ms) | `shortcut route 12.9716,77.5946 12.2958,76.6394 -algo X`. A single cold query, so the CH time includes first-touch page faults | 2026-11-17 |

## Architecture

- `osm.go` reads car-routable ways from an `.osm.pbf` in two passes (ways, then
  the coordinates of the nodes they use). Edge weight is travel time in ms from
  a fixed speed per highway type. It handles one-way streets, roundabouts and
  `access=no/private`.
- `prepare.go` keeps the largest strongly connected component (Tarjan with an
  explicit stack, since road graphs are millions of nodes deep) and renumbers
  nodes in k-d tree order. The tree lives in the numbering itself: the node in
  the middle of any id range splits that range. `Nearest` walks it on unit-sphere
  coordinates, where straight-line nearest is also great-circle nearest.
- `graph.go` stores the graph as compressed sparse row arrays and saves it as a
  flat binary file.
- `dijkstra.go` is Dijkstra with an indexed binary heap (decrease-key) and
  per-query stamps, so a query doesn't clear O(n) state. Given a potential (a
  lower bound on the distance left) the same loop is A*. The A* bound is
  straight-line distance at the top road speed. Edge weights are rounded up at
  import so that bound never overshoots a real road.
- `alt.go` is ALT: A* with landmark bounds. Eight landmarks are picked far apart
  by farthest-point selection, and exact distances to and from each are stored for
  every node. The triangle inequality turns them into a lower bound that follows
  the actual roads.
- `bidi.go` is bidirectional search, used two ways: plain bidirectional
  Dijkstra (stop once the two frontiers' smallest keys add up to the best
  meeting), and the CH query.
- `ch.go` is Contraction Hierarchies. Nodes are contracted one at a time in order
  of edge difference, contracted neighbors and level, with lazy priority updates.
  Removing a node adds a shortcut between two neighbors unless a bounded witness
  search finds another path at least as short. A query is a bidirectional search
  that only climbs in rank. Shortcuts record the node they skip, and are unpacked
  recursively into real roads.

## Status

Works: OSM import, largest-SCC filtering, k-d tree snapping, Dijkstra, bidirectional
Dijkstra, A*, ALT and Contraction Hierarchies, with `import`, `landmarks`, `contract`,
`route` and `bench` commands. Each piece is tested against a brute-force oracle on
random graphs: Dijkstra against Bellman-Ford, every faster algorithm against
Dijkstra (distance and a valid path with that weight), the SCC against all-pairs
reachability, `Nearest` against a linear haversine scan, and Prepare against the
original graph's distances. Deliberately broken versions of the A* bound, the
bidirectional stopping rule and the CH witness check all fail those tests.
`bench -check` does the same comparison on the real graph.

Not handled yet: turn restrictions and `maxspeed` tags. Contraction is single-threaded. `bench -cpuprofile` writes a CPU profile; on CH queries the remaining time is split between the search, unpacking shortcuts (binary search per half) and the allocator.
