package shortcut

import (
	"math/rand/v2"
	"path/filepath"
	"reflect"
	"testing"

	"github.com/paulmach/osm"
)

func randomGraph(r *rand.Rand, n, m int) *Graph {
	lat, lon := make([]float64, n), make([]float64, n)
	edges := make([]Edge, m)
	for i := range edges {
		edges[i] = Edge{uint32(r.IntN(n)), uint32(r.IntN(n)), uint32(1 + r.IntN(100))}
	}
	return NewGraph(lat, lon, edges)
}

// bellmanFord is the slow, obviously-correct oracle.
func bellmanFord(g *Graph, s uint32) []uint32 {
	dist := make([]uint32, g.NumNodes())
	for i := range dist {
		dist[i] = Inf
	}
	dist[s] = 0
	for changed := true; changed; {
		changed = false
		for u := range g.NumNodes() {
			if dist[u] == Inf {
				continue
			}
			for i := g.FirstOut[u]; i < g.FirstOut[u+1]; i++ {
				if nd := dist[u] + g.Weight[i]; nd < dist[g.Head[i]] {
					dist[g.Head[i]], changed = nd, true
				}
			}
		}
	}
	return dist
}

func pathWeight(t *testing.T, g *Graph, path []uint32) uint32 {
	var sum uint32
	for k := 1; k < len(path); k++ {
		best := uint32(Inf)
		for i := g.FirstOut[path[k-1]]; i < g.FirstOut[path[k-1]+1]; i++ {
			if g.Head[i] == path[k] && g.Weight[i] < best {
				best = g.Weight[i]
			}
		}
		if best == Inf {
			t.Fatalf("path uses missing edge %d->%d", path[k-1], path[k])
		}
		sum += best
	}
	return sum
}

func TestDijkstraMatchesBellmanFord(t *testing.T) {
	r := rand.New(rand.NewPCG(1, 2))
	for trial := range 200 {
		n := 2 + r.IntN(60)
		g := randomGraph(r, n, r.IntN(4*n))
		d := NewDijkstra(g)
		s := uint32(r.IntN(n))
		want := bellmanFord(g, s)
		d.Route(s, Inf)
		for v := range n {
			if got := d.Dist(uint32(v)); got != want[v] {
				t.Fatalf("trial %d: dist(%d->%d) = %d, want %d", trial, s, v, got, want[v])
			}
		}
		// Point-to-point queries reuse the same Dijkstra and must stay correct.
		for range 10 {
			tgt := uint32(r.IntN(n))
			got, path := d.Route(s, tgt)
			if got != want[tgt] {
				t.Fatalf("trial %d: route(%d,%d) = %d, want %d", trial, s, tgt, got, want[tgt])
			}
			if got != Inf {
				if path[0] != s || path[len(path)-1] != tgt || pathWeight(t, g, path) != got {
					t.Fatalf("trial %d: bad path %v for dist %d", trial, path, got)
				}
			}
		}
	}
}

func TestSaveLoad(t *testing.T) {
	r := rand.New(rand.NewPCG(3, 4))
	g := randomGraph(r, 50, 200)
	for i := range g.Lat {
		g.Lat[i], g.Lon[i] = r.Float64()*90, r.Float64()*180
	}
	p := filepath.Join(t.TempDir(), "g.bin")
	if err := g.Save(p); err != nil {
		t.Fatal(err)
	}
	h, err := Load(p)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(g, h) {
		t.Fatal("graph changed after save/load")
	}
}

func TestWayRule(t *testing.T) {
	tags := func(kv ...string) osm.Tags {
		var ts osm.Tags
		for i := 0; i < len(kv); i += 2 {
			ts = append(ts, osm.Tag{Key: kv[i], Value: kv[i+1]})
		}
		return ts
	}
	cases := []struct {
		tags     osm.Tags
		fwd, bwd bool
	}{
		{tags("highway", "residential"), true, true},
		{tags("highway", "residential", "oneway", "yes"), true, false},
		{tags("highway", "residential", "oneway", "-1"), false, true},
		{tags("highway", "motorway"), true, false},
		{tags("highway", "motorway", "oneway", "no"), true, true},
		{tags("highway", "primary", "junction", "roundabout"), true, false},
		{tags("highway", "service", "access", "private"), false, false},
		{tags("highway", "footway"), false, false},
		{tags("building", "yes"), false, false},
	}
	for _, c := range cases {
		if _, f, b := wayRule(c.tags); f != c.fwd || b != c.bwd {
			t.Errorf("%v: got fwd=%v bwd=%v, want %v %v", c.tags, f, b, c.fwd, c.bwd)
		}
	}
}
