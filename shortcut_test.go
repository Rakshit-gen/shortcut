package shortcut

import (
	"math"
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
	g := NewGraph(n, edges)
	g.Lat, g.Lon = lat, lon
	return g
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

func withCoords(r *rand.Rand, g *Graph) *Graph {
	for i := range g.Lat {
		g.Lat[i], g.Lon[i] = 8+r.Float64()*10, 74+r.Float64()*10 // roughly South India
	}
	return g
}

// reach[u][v] reports whether v is reachable from u, by BFS from every node.
func reachability(g *Graph) [][]bool {
	n := g.NumNodes()
	reach := make([][]bool, n)
	for s := range n {
		reach[s] = make([]bool, n)
		reach[s][s] = true
		queue := []uint32{uint32(s)}
		for len(queue) > 0 {
			u := queue[0]
			queue = queue[1:]
			for i := g.FirstOut[u]; i < g.FirstOut[u+1]; i++ {
				if v := g.Head[i]; !reach[s][v] {
					reach[s][v] = true
					queue = append(queue, v)
				}
			}
		}
	}
	return reach
}

func TestLargestSCC(t *testing.T) {
	r := rand.New(rand.NewPCG(5, 6))
	for trial := range 300 {
		n := 1 + r.IntN(40)
		g := randomGraph(r, n, r.IntN(3*n))
		reach := reachability(g)
		wantSize := 0
		for v := range n {
			size := 0
			for u := range n {
				if reach[v][u] && reach[u][v] {
					size++
				}
			}
			wantSize = max(wantSize, size)
		}
		keep := largestSCC(g)
		var got []int
		for v, k := range keep {
			if k {
				got = append(got, v)
			}
		}
		if len(got) != wantSize {
			t.Fatalf("trial %d: SCC size %d, want %d", trial, len(got), wantSize)
		}
		for _, a := range got {
			for _, b := range got {
				if !reach[a][b] {
					t.Fatalf("trial %d: %d cannot reach %d inside the SCC", trial, a, b)
				}
			}
		}
	}
}

func TestPrepareKeepsDistances(t *testing.T) {
	r := rand.New(rand.NewPCG(7, 8))
	for trial := range 100 {
		n := 2 + r.IntN(80)
		g := withCoords(r, randomGraph(r, n, 1+r.IntN(4*n)))
		p := g.Prepare()
		// Coordinates are unique random floats, so they identify nodes.
		old := map[[2]float64]uint32{}
		for v := range g.Lat {
			old[[2]float64{g.Lat[v], g.Lon[v]}] = uint32(v)
		}
		od, pd := NewDijkstra(g), NewDijkstra(p)
		for s := range p.NumNodes() {
			pd.Route(uint32(s), Inf)
			od.Route(old[[2]float64{p.Lat[s], p.Lon[s]}], Inf)
			for v := range p.NumNodes() {
				got, want := pd.Dist(uint32(v)), od.Dist(old[[2]float64{p.Lat[v], p.Lon[v]}])
				if got == Inf {
					t.Fatalf("trial %d: %d cannot reach %d after Prepare", trial, s, v)
				}
				if got != want {
					t.Fatalf("trial %d: dist %d->%d = %d, want %d", trial, s, v, got, want)
				}
			}
		}
	}
}

func TestNearestMatchesLinearScan(t *testing.T) {
	r := rand.New(rand.NewPCG(9, 10))
	for _, n := range []int{1, 2, 3, 10, 1000, 20000} {
		g0 := NewGraph(n, nil)
		g0.Lat, g0.Lon = make([]float64, n), make([]float64, n)
		g := withCoords(r, g0)
		order := make([]uint32, n)
		for i := range order {
			order[i] = uint32(i)
		}
		pts := make([][3]float64, n)
		for v := range pts {
			pts[v] = toXYZ(g.Lat[v], g.Lon[v])
		}
		kdOrder(pts, order, 0) // no edges, so build the k-d layout without the SCC step
		g = g.permute(order)
		for range 500 {
			lat, lon := 7+r.Float64()*12, 73+r.Float64()*12
			best := math.Inf(1)
			for v := range g.Lat {
				best = math.Min(best, Haversine(lat, lon, g.Lat[v], g.Lon[v]))
			}
			v := g.Nearest(lat, lon)
			if got := Haversine(lat, lon, g.Lat[v], g.Lon[v]); got > best+1e-6 {
				t.Fatalf("n=%d: nearest is %.3f m away, linear scan found %.3f m", n, got, best)
			}
		}
	}
}

// geoGraph is a random graph whose weights behave like road travel times:
// every edge takes at least its straight-line length at maxKmh, like import.
func geoGraph(r *rand.Rand, n, m int) *Graph {
	g := withCoords(r, randomGraph(r, n, 0))
	var edges []Edge
	for range m {
		a, b := uint32(r.IntN(n)), uint32(r.IntN(n))
		// Mostly short edges between nearby ids, like a road network after
		// k-d renumbering, plus a few long ones.
		if r.IntN(4) > 0 {
			b = uint32(min(n-1, int(a)+1+r.IntN(5)))
		}
		kmh := 10 + r.Float64()*(maxKmh-10)
		w := uint32(math.Max(1, math.Ceil(Haversine(g.Lat[a], g.Lon[a], g.Lat[b], g.Lon[b])/(kmh/3.6)*1000)))
		edges = append(edges, Edge{a, b, w}, Edge{b, a, w + uint32(r.IntN(3))})
	}
	h := NewGraph(n, edges)
	h.Lat, h.Lon = g.Lat, g.Lon
	return h.Prepare()
}

type router interface {
	Route(s, t uint32) (uint32, []uint32)
	Settled() int
}

func checkRouter(t *testing.T, name string, g *Graph, rt router, r *rand.Rand, queries int) {
	t.Helper()
	d := NewDijkstra(g)
	n := g.NumNodes()
	for range queries {
		s, tg := uint32(r.IntN(n)), uint32(r.IntN(n))
		want, _ := d.Route(s, tg)
		got, path := rt.Route(s, tg)
		if got != want {
			t.Fatalf("%s: route(%d,%d) = %d, want %d", name, s, tg, got, want)
		}
		if got == Inf {
			continue
		}
		if path[0] != s || path[len(path)-1] != tg || pathWeight(t, g, path) != got {
			t.Fatalf("%s: route(%d,%d): path %v does not have weight %d", name, s, tg, path, got)
		}
	}
}

func TestRoutersMatchDijkstra(t *testing.T) {
	r := rand.New(rand.NewPCG(11, 12))
	for range 60 {
		n := 2 + r.IntN(300)
		g := geoGraph(r, n, 3*n)
		checkRouter(t, "bidi", g, NewBidiDijkstra(g, g.Reverse()), r, 50)
		checkRouter(t, "astar", g, NewAStar(g), r, 50)
		checkRouter(t, "alt", g, NewALT(g, SelectLandmarks(g, 1+r.IntN(6), r.Uint64())), r, 50)
	}
}

func TestLandmarksSaveLoad(t *testing.T) {
	r := rand.New(rand.NewPCG(13, 14))
	g := geoGraph(r, 200, 600)
	lm := SelectLandmarks(g, 4, 1)
	p := filepath.Join(t.TempDir(), "g.lm")
	if err := lm.Save(p); err != nil {
		t.Fatal(err)
	}
	got, err := LoadLandmarks(p)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(lm, got) {
		t.Fatal("landmarks changed after save/load")
	}
}

func TestCHMatchesDijkstra(t *testing.T) {
	r := rand.New(rand.NewPCG(15, 16))
	for range 60 {
		n := 2 + r.IntN(300)
		g := geoGraph(r, n, 3*n)
		checkRouter(t, "ch geo", g, NewCHRouter(Contract(g, nil)), r, 100)
	}
	// Raw random graphs: parallel edges, self-loops, unreachable pairs.
	for range 200 {
		n := 1 + r.IntN(40)
		g := randomGraph(r, n, r.IntN(4*n))
		checkRouter(t, "ch random", g, NewCHRouter(Contract(g, nil)), r, 100)
	}
}

func TestCHSaveLoad(t *testing.T) {
	r := rand.New(rand.NewPCG(17, 18))
	g := geoGraph(r, 300, 900)
	ch := Contract(g, nil)
	p := filepath.Join(t.TempDir(), "g.ch")
	if err := ch.Save(p); err != nil {
		t.Fatal(err)
	}
	got, err := LoadCH(p)
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(ch, got) {
		t.Fatal("hierarchy changed after save/load")
	}
}
