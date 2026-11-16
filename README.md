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
bin/shortcut route south.graph 12.9716,77.5946 12.2958,76.6394   # Bengaluru to Mysuru
bin/shortcut bench south.graph -n 1000
```

## Measured vs Claimed

Every number below states what it measures and how. Update this table
instead of writing new numbers straight into prose elsewhere (portfolio site,
resume). Keep one source of truth and link to it.

| Claim | Value | How measured | Date |
|---|---|---|---|
| Graph size | 26,863,660 nodes, 55,854,321 edges (largest SCC, 99.4% of imported nodes) | `shortcut import`, Geofabrik southern-zone extract (534 MB), Apple M4, 16 GB | 2026-11-17 |
| Import time / peak memory | 57 s / 4.5 GB RSS | `/usr/bin/time -l shortcut import`, same machine | 2026-11-17 |
| Dijkstra p50 / p99 query | 2.57 s / 5.62 s | `shortcut bench -n 100 -seed 1`, random node pairs, all reachable | 2026-11-17 |
| Dijkstra nodes settled, avg | 13.3M | same run | 2026-11-17 |
| Bengaluru to Mysuru | 1h33m free-flow, 481 ms query, 2.08M settled | `shortcut route 12.9716,77.5946 12.2958,76.6394` | 2026-11-17 |

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
  per-query stamps, so a query doesn't clear O(n) state.

## Status

Works: OSM import, largest-SCC filtering, k-d tree snapping, Dijkstra, and the
`import`, `route` and `bench` commands. Each piece is tested against a brute-force
oracle on random graphs: Dijkstra against Bellman-Ford, the SCC against all-pairs
reachability, `Nearest` against a linear haversine scan, and Prepare against the
original graph's distances.

Not handled yet: turn restrictions and `maxspeed` tags.
