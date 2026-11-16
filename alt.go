package shortcut

import "math/rand/v2"

// Landmarks holds exact distances to and from K landmark nodes, interleaved
// per node (index v*K+i) so one potential lookup reads two short runs.
// By the triangle inequality, for any landmark L:
//
//	d(v,t) >= d(L,t) - d(L,v)   and   d(v,t) >= d(v,L) - d(t,L)
//
// ALT uses the largest of these as its A* bound. Unlike the straight-line
// bound it knows about the actual roads, so it is much tighter.
type Landmarks struct {
	K     int
	Nodes []uint32
	From  []uint32 // d(L_i, v)
	To    []uint32 // d(v, L_i)
}

// SelectLandmarks picks k landmarks by farthest-point selection: each new
// landmark is the node farthest (by road) from all landmarks chosen so far,
// which spreads them toward the edges of the map.
func SelectLandmarks(g *Graph, k int, seed uint64) *Landmarks {
	n := g.NumNodes()
	lm := &Landmarks{K: k, From: make([]uint32, n*k), To: make([]uint32, n*k)}
	fwd, bwd := NewDijkstra(g), NewDijkstra(g.Reverse())
	near := make([]uint32, n) // distance from the closest chosen landmark
	for v := range near {
		near[v] = Inf
	}
	farthest := func() uint32 {
		best := uint32(0)
		for v := range near {
			if near[v] != Inf && (near[best] == Inf || near[v] > near[best]) {
				best = uint32(v)
			}
		}
		return best
	}
	// Seed from the node farthest from a random start.
	fwd.Route(uint32(rand.New(rand.NewPCG(seed, 0)).IntN(n)), Inf)
	for v := range near {
		near[v] = fwd.Dist(uint32(v))
	}
	l := farthest()
	for v := range near {
		near[v] = Inf
	}
	for i := range k {
		lm.Nodes = append(lm.Nodes, l)
		fwd.Route(l, Inf)
		bwd.Route(l, Inf)
		for v := range n {
			d := fwd.Dist(uint32(v))
			lm.From[v*k+i] = d
			lm.To[v*k+i] = bwd.Dist(uint32(v))
			near[v] = min(near[v], d)
		}
		l = farthest()
	}
	return lm
}

// bound is the ALT lower bound on d(v, t).
func (lm *Landmarks) bound(v, t uint32) uint32 {
	k := lm.K
	fv, ft := lm.From[int(v)*k:int(v)*k+k], lm.From[int(t)*k:int(t)*k+k]
	tv, tt := lm.To[int(v)*k:int(v)*k+k], lm.To[int(t)*k:int(t)*k+k]
	var best int64
	for i := range k {
		if ft[i] != Inf && fv[i] != Inf {
			best = max(best, int64(ft[i])-int64(fv[i]))
		}
		if tv[i] != Inf && tt[i] != Inf {
			best = max(best, int64(tv[i])-int64(tt[i]))
		}
	}
	return uint32(best)
}

// NewALT is A* with landmark bounds.
func NewALT(g *Graph, lm *Landmarks) *Dijkstra {
	d := NewDijkstra(g)
	d.pot = func(v uint32) uint32 { return lm.bound(v, d.target) }
	return d
}

var lmMagic = [8]byte{'s', 'h', 'c', 't', 'l', 'm', 'k', '1'}

func (lm *Landmarks) Save(path string) error {
	return writeBin(path, lmMagic, uint64(lm.K), uint64(len(lm.From)/lm.K), lm.Nodes, lm.From, lm.To)
}

func LoadLandmarks(path string) (*Landmarks, error) {
	lm := &Landmarks{}
	err := readBin(path, lmMagic, func(read func(...any)) {
		var k, n uint64
		read(&k, &n)
		lm.K = int(k)
		lm.Nodes, lm.From, lm.To = make([]uint32, k), make([]uint32, n*k), make([]uint32, n*k)
		read(lm.Nodes, lm.From, lm.To)
	})
	return lm, err
}
