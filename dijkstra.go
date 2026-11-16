package shortcut

import "math"

const Inf = math.MaxUint32

// Dijkstra runs point-to-point queries on one graph. Reuse it across queries:
// per-node state is reset lazily with a query stamp instead of being cleared.
type Dijkstra struct {
	g       *Graph
	dist    []uint32
	parent  []uint32
	stamp   []uint32
	cur     uint32
	heap    idxHeap
	Settled int // nodes settled by the last query
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

func (d *Dijkstra) distOf(v uint32) uint32 {
	if d.stamp[v] != d.cur {
		return Inf
	}
	return d.dist[v]
}

// Route returns the travel time from s to t in ms (Inf if unreachable) and
// the node path. Pass t = Inf to run a full one-to-all search.
func (d *Dijkstra) Route(s, t uint32) (uint32, []uint32) {
	d.cur++
	d.heap.clear()
	d.Settled = 0
	d.stamp[s], d.dist[s], d.parent[s] = d.cur, 0, Inf
	d.heap.push(s, 0)
	g := d.g
	for d.heap.len() > 0 {
		u, du := d.heap.pop()
		d.Settled++
		if u == t {
			break
		}
		for i := g.FirstOut[u]; i < g.FirstOut[u+1]; i++ {
			v, nd := g.Head[i], du+g.Weight[i]
			if nd < d.distOf(v) {
				if d.stamp[v] != d.cur {
					d.stamp[v] = d.cur
					d.heap.push(v, nd)
				} else {
					d.heap.decrease(v, nd)
				}
				d.dist[v], d.parent[v] = nd, u
			}
		}
	}
	if t == Inf || d.distOf(t) == Inf {
		return Inf, nil
	}
	var path []uint32
	for v := t; v != Inf; v = d.parent[v] {
		path = append(path, v)
	}
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return d.dist[t], path
}

// Dist returns the distance to v found by the last query.
func (d *Dijkstra) Dist(v uint32) uint32 { return d.distOf(v) }

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
