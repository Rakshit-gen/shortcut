package shortcut

import (
	"slices"
	"sort"
)

// CH is a contraction hierarchy. Every node has a rank (the order it was
// contracted in). Up holds edges u->w with rank(u) < rank(w). Down is stored
// at the lower end: an entry at w with head u is the edge u->w, rank(u) >
// rank(w). A query searches Up from s and Down from t, so both sides only
// climb, and they meet at the highest node of the shortest path.
//
// Mid is the contracted node a shortcut skips over, or Inf for a real road.
type CH struct {
	Up, Down       *Graph
	UpMid, DownMid []uint32
}

type arc struct{ to, w, mid uint32 }

type contractor struct {
	out, in        [][]arc // remaining graph: only nodes not yet contracted
	deleted, level []int32
	wdist, wstamp  []uint32
	wcur           uint32
	wheap          idxHeap
	up, down       []Edge
	upMid, downMid []uint32
	shortcuts      int
}

const (
	simLimit      = 50   // settled nodes per witness search when estimating priority
	contractLimit = 1000 // settled nodes per witness search when contracting
)

// Contract builds a contraction hierarchy. progress, if not nil, is called
// every million contracted nodes with the number of nodes left.
func Contract(g *Graph, progress func(left, shortcuts int)) *CH {
	n := g.NumNodes()
	c := &contractor{
		out:     make([][]arc, n),
		in:      make([][]arc, n),
		deleted: make([]int32, n),
		level:   make([]int32, n),
		wdist:   make([]uint32, n),
		wstamp:  make([]uint32, n),
		wheap:   newIdxHeap(n),
	}
	for u := range n {
		for i := g.FirstOut[u]; i < g.FirstOut[u+1]; i++ {
			if v := g.Head[i]; v != uint32(u) {
				c.addArc(uint32(u), v, g.Weight[i], Inf)
			}
		}
	}
	pq := newIdxHeap(n)
	for v := range n {
		pq.push(uint32(v), c.priority(uint32(v)))
	}
	for done := 1; pq.len() > 0; done++ {
		v, _ := pq.pop()
		// Lazy update: the stored priority may be stale. If v's real priority
		// is worse than the next node's, put it back and try again.
		if p := c.priority(v); pq.len() > 0 && p > pq.top() {
			pq.push(v, p)
			done--
			continue
		}
		for _, x := range c.contract(v) {
			c.deleted[x]++
			c.level[x] = max(c.level[x], c.level[v]+1)
			pq.update(x, c.priority(x))
		}
		if progress != nil && done%(1<<20) == 0 {
			progress(pq.len(), c.shortcuts)
		}
	}
	return c.build(n)
}

// priority orders contraction: prefer nodes that add few shortcuts relative
// to the edges they remove (edge difference), whose neighbors haven't been
// contracted much yet, and that sit low in the hierarchy so far. This keeps
// contraction spread out evenly across the map.
func (c *contractor) priority(v uint32) uint32 {
	added := c.addShortcuts(v, false)
	p := 2*(added-len(c.in[v])-len(c.out[v])) + int(c.deleted[v]) + int(c.level[v])
	return uint32(p + 1<<30)
}

// addShortcuts finds, for each in-neighbor u and out-neighbor w of v, whether
// u->v->w is the only shortest path from u to w once v is gone. If so it needs
// a shortcut. It returns the count and, with apply, adds them.
func (c *contractor) addShortcuts(v uint32, apply bool) int {
	limit := simLimit
	if apply {
		limit = contractLimit
	}
	count := 0
	for _, a := range c.in[v] {
		u := a.to
		var maxOut uint32
		for _, b := range c.out[v] {
			if b.to != u {
				maxOut = max(maxOut, b.w)
			}
		}
		if maxOut == 0 {
			continue // v's only out-neighbor is u
		}
		c.witness(u, v, a.w+maxOut, limit)
		for _, b := range c.out[v] {
			if b.to == u {
				continue
			}
			via := a.w + b.w
			if c.wstamp[b.to] == c.wcur && c.wdist[b.to] <= via {
				continue // a path at least as short avoids v
			}
			count++
			if apply {
				c.addArc(u, b.to, via, v)
			}
		}
	}
	return count
}

// witness runs a Dijkstra from src that never enters skip and gives up past
// maxDist or after limit settled nodes. Giving up early only costs extra
// shortcuts, never a wrong answer.
func (c *contractor) witness(src, skip, maxDist uint32, limit int) {
	c.wcur++
	c.wheap.clear()
	c.wstamp[src], c.wdist[src] = c.wcur, 0
	c.wheap.push(src, 0)
	for settled := 0; c.wheap.len() > 0 && settled < limit; settled++ {
		u, du := c.wheap.pop()
		if du > maxDist {
			return
		}
		for _, a := range c.out[u] {
			x, nd := a.to, du+a.w
			if x == skip || nd > maxDist {
				continue
			}
			if c.wstamp[x] != c.wcur {
				c.wstamp[x], c.wdist[x] = c.wcur, nd
				c.wheap.push(x, nd)
			} else if nd < c.wdist[x] && c.wheap.pos[x] >= 0 {
				c.wdist[x] = nd
				c.wheap.decrease(x, nd)
			}
		}
	}
}

// addArc adds u->w, or lowers its weight if a longer u->w is there. Keeping
// one arc per pair means a shortcut's two halves can be looked up by endpoints.
func (c *contractor) addArc(u, w, weight, mid uint32) {
	for i := range c.out[u] {
		if c.out[u][i].to != w {
			continue
		}
		if weight < c.out[u][i].w {
			c.out[u][i] = arc{w, weight, mid}
			for j := range c.in[w] {
				if c.in[w][j].to == u {
					c.in[w][j] = arc{u, weight, mid}
				}
			}
		}
		return
	}
	c.out[u] = append(c.out[u], arc{w, weight, mid})
	c.in[w] = append(c.in[w], arc{u, weight, mid})
}

func removeArc(list []arc, x uint32) []arc {
	for i := range list {
		if list[i].to == x {
			list[i] = list[len(list)-1]
			return list[:len(list)-1]
		}
	}
	return list
}

// contract adds v's shortcuts, moves v's arcs into the hierarchy, removes v
// from the remaining graph and returns its neighbors.
func (c *contractor) contract(v uint32) []uint32 {
	c.shortcuts += c.addShortcuts(v, true)
	var nbrs []uint32
	for _, b := range c.out[v] {
		c.up = append(c.up, Edge{v, b.to, b.w})
		c.upMid = append(c.upMid, b.mid)
		c.in[b.to] = removeArc(c.in[b.to], v)
		nbrs = append(nbrs, b.to)
	}
	for _, a := range c.in[v] {
		c.down = append(c.down, Edge{v, a.to, a.w})
		c.downMid = append(c.downMid, a.mid)
		c.out[a.to] = removeArc(c.out[a.to], v)
		nbrs = append(nbrs, a.to)
	}
	c.out[v], c.in[v] = nil, nil
	slices.Sort(nbrs)
	return slices.Compact(nbrs)
}

func (c *contractor) build(n int) *CH {
	ch := &CH{}
	ch.Up, ch.UpMid = sortedCSR(n, c.up, c.upMid)
	ch.Down, ch.DownMid = sortedCSR(n, c.down, c.downMid)
	return ch
}

// sortedCSR builds a CSR graph whose out-edges are sorted by head within each
// node, carrying mids along, so unpack can find a shortcut half by binary search.
func sortedCSR(n int, edges []Edge, mids []uint32) (*Graph, []uint32) {
	pos := make([]uint32, len(edges))
	g := csr(n, edges, pos)
	m := make([]uint32, len(edges))
	for i, p := range pos {
		m[p] = mids[i]
	}
	for v := range n {
		lo, hi := g.FirstOut[v], g.FirstOut[v+1]
		sort.Sort(segment{g.Head[lo:hi], g.Weight[lo:hi], m[lo:hi]})
	}
	return g, m
}

type segment struct{ head, w, mid []uint32 }

func (s segment) Len() int           { return len(s.head) }
func (s segment) Less(i, j int) bool { return s.head[i] < s.head[j] }
func (s segment) Swap(i, j int) {
	s.head[i], s.head[j] = s.head[j], s.head[i]
	s.w[i], s.w[j] = s.w[j], s.w[i]
	s.mid[i], s.mid[j] = s.mid[j], s.mid[i]
}

// CHRouter answers queries on a contraction hierarchy.
type CHRouter struct {
	ch       *CH
	f, b     *search
	idx, buf []uint32 // reused across queries to keep allocation to one per route
}

func NewCHRouter(ch *CH) *CHRouter {
	return &CHRouter{ch: ch, f: newSearch(ch.Up), b: newSearch(ch.Down)}
}

func (r *CHRouter) Settled() int { return r.f.settled + r.b.settled }

func (r *CHRouter) SetTrace(on bool) { r.f.trace, r.b.trace = on, on }

func (r *CHRouter) SettledNodes() []uint32 { return append(slices.Clone(r.f.traced), r.b.traced...) }

func (r *CHRouter) Route(s, t uint32) (uint32, []uint32) {
	best, meet := bidi(r.f, r.b, s, t, true)
	if best == Inf {
		return Inf, nil
	}
	// Up edges from s to the meeting node, collected backward.
	idx := r.idx[:0]
	for v := meet; v != s; v = r.f.parent[v] {
		idx = append(idx, r.f.pedge[v])
	}
	path := append(r.buf[:0], s)
	u := s
	for i := len(idx) - 1; i >= 0; i-- {
		e := idx[i]
		path = r.ch.unpack(path, u, r.ch.Up.Head[e], r.ch.UpMid[e])
		u = r.ch.Up.Head[e]
	}
	// Down edges from the meeting node to t: parent is the next node toward t.
	for v := meet; v != t; {
		y := r.b.parent[v]
		path = r.ch.unpack(path, v, y, r.ch.DownMid[r.b.pedge[v]])
		v = y
	}
	r.idx, r.buf = idx, path
	return best, slices.Clone(path)
}

// unpack appends the road nodes after u on the edge u->x.
func (ch *CH) unpack(path []uint32, u, x, mid uint32) []uint32 {
	if mid == Inf {
		return append(path, x)
	}
	// mid was contracted before u and x, so u->mid is a Down entry at mid
	// and mid->x is an Up entry at mid.
	path = ch.unpack(path, u, mid, edgeMid(ch.Down, ch.DownMid, mid, u))
	return ch.unpack(path, mid, x, edgeMid(ch.Up, ch.UpMid, mid, x))
}

func edgeMid(g *Graph, mids []uint32, at, head uint32) uint32 {
	lo, hi := g.FirstOut[at], g.FirstOut[at+1]
	heads := g.Head[lo:hi]
	if i, ok := slices.BinarySearch(heads, head); ok {
		return mids[lo+uint32(i)]
	}
	panic("contraction hierarchy is missing a shortcut half")
}

// Version 2: each node's edges are sorted by head.
var chMagic = [8]byte{'s', 'h', 'c', 't', 'c', 'h', 'h', '2'}

func (ch *CH) Save(path string) error {
	return writeBin(path, chMagic, uint64(ch.Up.NumNodes()), uint64(ch.Up.NumEdges()), uint64(ch.Down.NumEdges()),
		ch.Up.FirstOut, ch.Up.Head, ch.Up.Weight, ch.UpMid,
		ch.Down.FirstOut, ch.Down.Head, ch.Down.Weight, ch.DownMid)
}

func LoadCH(path string) (*CH, error) {
	ch := &CH{Up: &Graph{}, Down: &Graph{}}
	err := readBin(path, chMagic, func(read func(...any)) {
		var n, nu, nd uint64
		read(&n, &nu, &nd)
		for _, p := range []struct {
			g   *Graph
			mid *[]uint32
			m   uint64
		}{{ch.Up, &ch.UpMid, nu}, {ch.Down, &ch.DownMid, nd}} {
			p.g.FirstOut, p.g.Head, p.g.Weight = make([]uint32, n+1), make([]uint32, p.m), make([]uint32, p.m)
			*p.mid = make([]uint32, p.m)
			read(p.g.FirstOut, p.g.Head, p.g.Weight, *p.mid)
		}
	})
	return ch, err
}
