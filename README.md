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
| Graph size | 27,038,811 nodes, 56,187,921 edges | `shortcut import`, Geofabrik southern-zone extract (534 MB), Apple M4, 16 GB | 2026-11-17 |
| Import time / peak memory | 43 s / 3.5 GB RSS | `/usr/bin/time -l shortcut import`, same machine | 2026-11-17 |
| Dijkstra p50 / p99 query | 3.08 s / 8.61 s | `shortcut bench -n 100 -seed 1`, random node pairs, 1 of 100 unreachable | 2026-11-17 |
| Dijkstra nodes settled, avg | 13.6M | same run | 2026-11-17 |
| Bengaluru to Mysuru | 1h33m free-flow, 439 ms query, 2.08M settled | `shortcut route 12.9716,77.5946 12.2958,76.6394` | 2026-11-17 |

## Architecture

- `osm.go` reads car-routable ways from an `.osm.pbf` in two passes (ways, then
  the coordinates of the nodes they use). Edge weight is travel time in ms from
  a fixed speed per highway type. It handles one-way streets, roundabouts and
  `access=no/private`.
- `graph.go` stores the graph as compressed sparse row arrays and saves it as a
  flat binary file.
- `dijkstra.go` is Dijkstra with an indexed binary heap (decrease-key) and
  per-query stamps, so a query doesn't clear O(n) state.

## Status

Works: OSM import, CSR graph, Dijkstra, and the `import`, `route` and `bench` commands.
Dijkstra is tested against Bellman-Ford on random graphs.

Not handled yet: turn restrictions, `maxspeed` tags, and keeping only the largest
strongly connected component (so random queries can hit unreachable pairs).
Snapping a coordinate to a node is a linear scan.
