package shortcut

import "math"

const Inf = math.MaxUint32

// Dijkstra runs point-to-point queries on one graph. Reuse it across queries:
// per-node state is reset lazily with a query stamp instead of being cleared.
//
// With a potential (a lower bound on the distance to the target) it becomes
// A* or ALT. The heap key is then dist + potential, and a node that was
// already settled is reopened if a shorter path to it turns up, so a bound
// that is admissible but not perfectly consistent still gives exact answers.
type Dijkstra struct {
	g       *Graph
	dist    []uint32
	parent  []uint32
	stamp   []uint32
	h       []uint32 // potential per node, filled the first time a query touches it
	cur     uint32
	target  uint32
	heap    idxHeap
	pot     func(v uint32) uint32
	settled int
	tracing bool
	trace   []uint32
}

func NewDijkstra(g *Graph) *Dijkstra {
	n := g.NumNodes()
	return &Dijkstra{
		g:      g,
		dist:   make([]uint32, n),
		parent: make([]uint32, n),
		stamp:  make([]uint32, n),
		heap:   newIdxHeap(n),
	}
}

// NewAStar is Dijkstra guided by straight-line distance at top road speed.
func NewAStar(g *Graph) *Dijkstra {
	d := NewDijkstra(g)
	d.pot = func(v uint32) uint32 {
		t := d.target
		m := Haversine(g.Lat[v], g.Lon[v], g.Lat[t], g.Lon[t])
		// Edges are at least m/maxKmh long (osm.go rounds up); the margin
		// absorbs floating point error in haversine.
		return uint32(m / (maxKmh / 3.6) * 1000 * (1 - 1e-6))
	}
	return d
}

func (d *Dijkstra) Settled() int { return d.settled }

// SetTrace turns on recording of settled nodes, for drawing a search.
func (d *Dijkstra) SetTrace(on bool) { d.tracing = on }

// SettledNodes lists the nodes the last query settled, if tracing is on.
func (d *Dijkstra) SettledNodes() []uint32 { return d.trace }

func (d *Dijkstra) distOf(v uint32) uint32 {
	if d.stamp[v] != d.cur {
		return Inf
	}
	return d.dist[v]
}

// Route returns the travel time from s to t in ms (Inf if unreachable) and
// the node path. Pass t = Inf to run a full one-to-all search (no potential).
func (d *Dijkstra) Route(s, t uint32) (uint32, []uint32) {
	d.cur++
	d.heap.clear()
	d.settled = 0
	d.target = t
	d.trace = d.trace[:0]
	pot := d.pot
	if t == Inf {
		pot = nil
	}
	if pot != nil && d.h == nil {
		d.h = make([]uint32, len(d.dist))
	}
	touch := func(v uint32) {
		d.stamp[v] = d.cur
		if pot != nil {
			d.h[v] = pot(v)
		}
	}
	key := func(v uint32) uint32 {
		if pot != nil {
			return d.dist[v] + d.h[v]
		}
		return d.dist[v]
	}

	touch(s)
	d.dist[s], d.parent[s] = 0, Inf
	d.heap.push(s, key(s))
	g := d.g
	for d.heap.len() > 0 {
		u, _ := d.heap.pop()
		d.settled++
		if d.tracing {
			d.trace = append(d.trace, u)
		}
		if u == t {
			break
		}
		du := d.dist[u]
		for i := g.FirstOut[u]; i < g.FirstOut[u+1]; i++ {
			v, nd := g.Head[i], du+g.Weight[i]
			if d.stamp[v] != d.cur {
				touch(v)
			} else if nd >= d.dist[v] {
				continue
			}
			d.dist[v], d.parent[v] = nd, u
			if d.heap.pos[v] >= 0 {
				d.heap.decrease(v, key(v))
			} else {
				d.heap.push(v, key(v))
			}
		}
	}
	if t == Inf || d.distOf(t) == Inf {
		return Inf, nil
	}
	return d.dist[t], walkBack(d.parent, t)
}

// Dist returns the distance to v found by the last query.
func (d *Dijkstra) Dist(v uint32) uint32 { return d.distOf(v) }

// walkBack follows parent pointers from v to the root and returns the path root..v.
func walkBack(parent []uint32, v uint32) []uint32 {
	var path []uint32
	for ; v != Inf; v = parent[v] {
		path = append(path, v)
	}
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}

// idxHeap is a binary min-heap over node ids with decrease-key.
type idxHeap struct {
	nodes []uint32
	keys  []uint32
	pos   []int32 // index of node in nodes, -1 if absent
}

func newIdxHeap(n int) idxHeap {
	pos := make([]int32, n)
	for i := range pos {
		pos[i] = -1
	}
	return idxHeap{pos: pos}
}

func (h *idxHeap) len() int { return len(h.nodes) }

// top returns the smallest key, or Inf if the heap is empty.
func (h *idxHeap) top() uint32 {
	if len(h.keys) == 0 {
		return Inf
	}
	return h.keys[0]
}

func (h *idxHeap) clear() {
	for _, v := range h.nodes {
		h.pos[v] = -1
	}
	h.nodes, h.keys = h.nodes[:0], h.keys[:0]
}

func (h *idxHeap) push(v, k uint32) {
	h.nodes = append(h.nodes, v)
	h.keys = append(h.keys, k)
	h.pos[v] = int32(len(h.nodes) - 1)
	h.up(len(h.nodes) - 1)
}

func (h *idxHeap) decrease(v, k uint32) {
	i := h.pos[v]
	h.keys[i] = k
	h.up(int(i))
}

// update sets v's key in either direction.
func (h *idxHeap) update(v, k uint32) {
	i := int(h.pos[v])
	old := h.keys[i]
	h.keys[i] = k
	if k < old {
		h.up(i)
	} else {
		h.down(i)
	}
}

func (h *idxHeap) pop() (uint32, uint32) {
	v, k := h.nodes[0], h.keys[0]
	last := len(h.nodes) - 1
	h.swap(0, last)
	h.nodes, h.keys = h.nodes[:last], h.keys[:last]
	h.pos[v] = -1
	if last > 0 {
		h.down(0)
	}
	return v, k
}

func (h *idxHeap) swap(i, j int) {
	h.nodes[i], h.nodes[j] = h.nodes[j], h.nodes[i]
	h.keys[i], h.keys[j] = h.keys[j], h.keys[i]
	h.pos[h.nodes[i]] = int32(i)
	h.pos[h.nodes[j]] = int32(j)
}

func (h *idxHeap) up(i int) {
	for i > 0 {
		p := (i - 1) / 2
		if h.keys[p] <= h.keys[i] {
			return
		}
		h.swap(i, p)
		i = p
	}
}

func (h *idxHeap) down(i int) {
	n := len(h.nodes)
	for {
		m, l, r := i, 2*i+1, 2*i+2
		if l < n && h.keys[l] < h.keys[m] {
			m = l
		}
		if r < n && h.keys[r] < h.keys[m] {
			m = r
		}
		if m == i {
			return
		}
		h.swap(i, m)
		i = m
	}
}
