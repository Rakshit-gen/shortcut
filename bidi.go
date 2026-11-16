package shortcut

import "slices"

// search is one side of a bidirectional query.
type search struct {
	g       *Graph
	dist    []uint32
	parent  []uint32
	pedge   []uint32 // index in g of the edge that reached the node
	stamp   []uint32
	cur     uint32
	heap    idxHeap
	settled int
	trace   bool
	traced  []uint32
}

func newSearch(g *Graph) *search {
	n := g.NumNodes()
	return &search{
		g:      g,
		dist:   make([]uint32, n),
		parent: make([]uint32, n),
		pedge:  make([]uint32, n),
		stamp:  make([]uint32, n),
		heap:   newIdxHeap(n),
	}
}

func (s *search) reset(src uint32) {
	s.cur++
	s.heap.clear()
	s.settled = 0
	s.traced = s.traced[:0]
	s.stamp[src], s.dist[src], s.parent[src], s.pedge[src] = s.cur, 0, Inf, Inf
	s.heap.push(src, 0)
}

func (s *search) distOf(v uint32) uint32 {
	if s.stamp[v] != s.cur {
		return Inf
	}
	return s.dist[v]
}

// step settles one node and relaxes its edges. Whenever a node gets a better
// distance and the other side has reached it too, the best meeting is updated.
func (s *search) step(other *search, best, meet *uint32) {
	u, du := s.heap.pop()
	s.settled++
	if s.trace {
		s.traced = append(s.traced, u)
	}
	g := s.g
	for i := g.FirstOut[u]; i < g.FirstOut[u+1]; i++ {
		v, nd := g.Head[i], du+g.Weight[i]
		if s.stamp[v] != s.cur {
			s.stamp[v] = s.cur
			s.dist[v] = nd
			s.heap.push(v, nd)
		} else if nd < s.dist[v] {
			s.dist[v] = nd
			s.heap.decrease(v, nd) // nonnegative weights: an improved node is still in the heap
		} else {
			continue
		}
		s.parent[v], s.pedge[v] = u, i
		if od := other.distOf(v); od != Inf && nd+od < *best {
			*best, *meet = nd+od, v
		}
	}
}

// bidi runs f from s and b from t, always growing the side with the smaller
// key, and returns the best distance and the node where the sides met.
//
// Plain bidirectional Dijkstra can stop once the two smallest keys add up to
// the best meeting. A contraction hierarchy only searches upward, so the
// sides can't be cut off that early: each side runs until its own smallest
// key reaches the best meeting.
func bidi(f, b *search, s, t uint32, ch bool) (best, meet uint32) {
	f.reset(s)
	b.reset(t)
	if s == t {
		return 0, s
	}
	best, meet = Inf, Inf
	for {
		ft, bt := f.heap.top(), b.heap.top()
		if ch {
			if ft >= best {
				ft = Inf
			}
			if bt >= best {
				bt = Inf
			}
			if ft == Inf && bt == Inf {
				break
			}
		} else if uint64(ft)+uint64(bt) >= uint64(best) {
			break
		}
		if ft <= bt {
			f.step(b, &best, &meet)
		} else {
			b.step(f, &best, &meet)
		}
	}
	return best, meet
}

// BidiDijkstra searches forward from s and backward from t at the same time.
type BidiDijkstra struct{ f, b *search }

// NewBidiDijkstra takes the graph and its Reverse.
func NewBidiDijkstra(g, rev *Graph) *BidiDijkstra {
	return &BidiDijkstra{newSearch(g), newSearch(rev)}
}

func (d *BidiDijkstra) Settled() int { return d.f.settled + d.b.settled }

// SetTrace turns on recording of settled nodes.
func (d *BidiDijkstra) SetTrace(on bool) { d.f.trace, d.b.trace = on, on }

func (d *BidiDijkstra) SettledNodes() []uint32 {
	return append(slices.Clone(d.f.traced), d.b.traced...)
}

func (d *BidiDijkstra) Route(s, t uint32) (uint32, []uint32) {
	best, meet := bidi(d.f, d.b, s, t, false)
	if best == Inf {
		return Inf, nil
	}
	path := walkBack(d.f.parent, meet)
	// On the reverse graph a node's parent is the next node toward t.
	for v := meet; v != t; {
		v = d.b.parent[v]
		path = append(path, v)
	}
	return best, path
}
