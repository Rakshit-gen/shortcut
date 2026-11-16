// Command shortcut imports OSM road data and answers routing queries.
package main

import (
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"math/rand/v2"
	"os"
	"runtime/pprof"
	"slices"
	"strconv"
	"strings"
	"time"

	"github.com/Rakshit-gen/shortcut"
)

const usage = `usage:
  shortcut import    <in.osm.pbf> <out.graph>
  shortcut landmarks <graph> [-k 8]          writes <graph>.lm for -algo alt
  shortcut contract  <graph>                 writes <graph>.ch for -algo ch
  shortcut route     <graph> <lat,lon> <lat,lon> [-algo name]
  shortcut bench     <graph> [-algo name] [-n queries] [-seed n] [-check]

algorithms: dijkstra, bidi, astar, alt, ch`

type router interface {
	Route(s, t uint32) (uint32, []uint32)
	Settled() int
}

func main() {
	if len(os.Args) < 3 {
		die(usage)
	}
	var err error
	switch os.Args[1] {
	case "import":
		err = importCmd(os.Args[2:])
	case "landmarks":
		err = landmarksCmd(os.Args[2:])
	case "contract":
		err = contractCmd(os.Args[2:])
	case "route":
		err = routeCmd(os.Args[2:])
	case "bench":
		err = benchCmd(os.Args[2:])
	default:
		die(usage)
	}
	if err != nil {
		die(err.Error())
	}
}

func die(msg string) {
	fmt.Fprintln(os.Stderr, msg)
	os.Exit(1)
}

func newRouter(path, algo string, g *shortcut.Graph) (router, error) {
	switch algo {
	case "dijkstra":
		return shortcut.NewDijkstra(g), nil
	case "bidi":
		return shortcut.NewBidiDijkstra(g, g.Reverse()), nil
	case "astar":
		return shortcut.NewAStar(g), nil
	case "alt":
		lm, err := shortcut.LoadLandmarks(path + ".lm")
		if err != nil {
			return nil, fmt.Errorf("%w (run: shortcut landmarks %s)", err, path)
		}
		return shortcut.NewALT(g, lm), nil
	case "ch":
		ch, err := shortcut.LoadCH(path + ".ch")
		if err != nil {
			return nil, fmt.Errorf("%w (run: shortcut contract %s)", err, path)
		}
		return shortcut.NewCHRouter(ch), nil
	}
	return nil, fmt.Errorf("unknown algorithm %q", algo)
}

func importCmd(args []string) error {
	if len(args) != 2 {
		return errors.New(usage)
	}
	start := time.Now()
	g, err := shortcut.ImportPBF(args[0])
	if err != nil {
		return err
	}
	fmt.Printf("%d nodes, %d edges in %s\n", g.NumNodes(), g.NumEdges(), time.Since(start).Round(time.Millisecond))
	return g.Save(args[1])
}

func landmarksCmd(args []string) error {
	fs := flag.NewFlagSet("landmarks", flag.ExitOnError)
	k := fs.Int("k", 8, "number of landmarks")
	fs.Parse(args[1:])
	g, err := shortcut.Load(args[0])
	if err != nil {
		return err
	}
	start := time.Now()
	lm := shortcut.SelectLandmarks(g, *k, 1)
	fmt.Printf("%d landmarks in %s\n", *k, time.Since(start).Round(time.Millisecond))
	return lm.Save(args[0] + ".lm")
}

func parseLatLon(s string) (float64, float64, error) {
	a, b, ok := strings.Cut(s, ",")
	lat, err1 := strconv.ParseFloat(a, 64)
	lon, err2 := strconv.ParseFloat(b, 64)
	if !ok || err1 != nil || err2 != nil {
		return 0, 0, fmt.Errorf("bad coordinate %q, want lat,lon", s)
	}
	return lat, lon, nil
}

func routeCmd(args []string) error {
	if len(args) < 3 {
		return errors.New(usage)
	}
	fs := flag.NewFlagSet("route", flag.ExitOnError)
	algo := fs.String("algo", "dijkstra", "routing algorithm")
	jsonOut := fs.String("json", "", "also write the route and the searched nodes to this file, for drawing")
	fs.Parse(args[3:])
	g, err := shortcut.Load(args[0])
	if err != nil {
		return err
	}
	lat1, lon1, err := parseLatLon(args[1])
	if err != nil {
		return err
	}
	lat2, lon2, err := parseLatLon(args[2])
	if err != nil {
		return err
	}
	rt, err := newRouter(args[0], *algo, g)
	if err != nil {
		return err
	}
	s, t := g.Nearest(lat1, lon1), g.Nearest(lat2, lon2)
	start := time.Now()
	ms, path := rt.Route(s, t)
	took := time.Since(start)
	if ms == shortcut.Inf {
		return fmt.Errorf("no route between nodes %d and %d", s, t)
	}
	fmt.Printf("%s: time %s, %d nodes on path, %d settled, query %s\n", *algo,
		time.Duration(ms)*time.Millisecond, len(path), rt.Settled(), took.Round(time.Microsecond))
	if *jsonOut == "" {
		return nil
	}
	return writeRouteJSON(*jsonOut, *algo, g, rt, s, t, ms, path, took)
}

func benchCmd(args []string) error {
	fs := flag.NewFlagSet("bench", flag.ExitOnError)
	algo := fs.String("algo", "dijkstra", "routing algorithm")
	n := fs.Int("n", 1000, "number of random queries")
	seed := fs.Uint64("seed", 1, "random seed")
	check := fs.Bool("check", false, "compare every answer with plain Dijkstra")
	profile := fs.String("cpuprofile", "", "write a CPU profile of the queries to this file")
	fs.Parse(args[1:])
	g, err := shortcut.Load(args[0])
	if err != nil {
		return err
	}
	rt, err := newRouter(args[0], *algo, g)
	if err != nil {
		return err
	}
	var oracle *shortcut.Dijkstra
	if *check {
		oracle = shortcut.NewDijkstra(g)
	}
	if *profile != "" {
		f, err := os.Create(*profile)
		if err != nil {
			return err
		}
		defer f.Close()
		pprof.StartCPUProfile(f)
		defer pprof.StopCPUProfile()
	}
	r := rand.New(rand.NewPCG(*seed, 0))
	times := make([]time.Duration, *n)
	settled, unreachable, wrong := 0, 0, 0
	for i := range times {
		s, t := uint32(r.IntN(g.NumNodes())), uint32(r.IntN(g.NumNodes()))
		start := time.Now()
		ms, _ := rt.Route(s, t)
		times[i] = time.Since(start)
		settled += rt.Settled()
		if ms == shortcut.Inf {
			unreachable++
		}
		if oracle != nil {
			if want, _ := oracle.Route(s, t); want != ms {
				wrong++
				fmt.Printf("mismatch: %d -> %d: got %d ms, dijkstra %d ms\n", s, t, ms, want)
			}
		}
	}
	slices.Sort(times)
	pct := func(p float64) time.Duration { return times[int(p*float64(len(times)-1))] }
	fmt.Printf("%s: %d queries, p50 %s, p99 %s, avg settled %d, unreachable %d",
		*algo, *n, pct(0.5), pct(0.99), settled / *n, unreachable)
	if *check {
		fmt.Printf(", %d differ from dijkstra", wrong)
	}
	fmt.Println()
	if wrong > 0 {
		return errors.New("answers differ from dijkstra")
	}
	return nil
}

func contractCmd(args []string) error {
	if len(args) != 1 {
		return errors.New(usage)
	}
	g, err := shortcut.Load(args[0])
	if err != nil {
		return err
	}
	start := time.Now()
	ch := shortcut.Contract(g, func(left, shortcuts int) {
		fmt.Printf("  %s: %d nodes left, %d shortcuts\n", time.Since(start).Round(time.Second), left, shortcuts)
	})
	fmt.Printf("contracted %d nodes in %s: %d up edges, %d down edges\n", g.NumNodes(),
		time.Since(start).Round(time.Second), ch.Up.NumEdges(), ch.Down.NumEdges())
	return ch.Save(args[0] + ".ch")
}

type tracer interface {
	SetTrace(bool)
	SettledNodes() []uint32
}

// writeRouteJSON reruns the query with tracing on (so tracing doesn't skew the
// timing printed above) and writes coordinates for the path, the settled
// nodes and a sample of the road network around them.
func writeRouteJSON(file, algo string, g *shortcut.Graph, rt router, s, t, ms uint32, path []uint32, took time.Duration) error {
	tr, ok := rt.(tracer)
	if !ok {
		return fmt.Errorf("%s cannot trace its search", algo)
	}
	tr.SetTrace(true)
	rt.Route(s, t)
	settled := tr.SettledNodes()

	pt := func(v uint32) [2]float64 { return [2]float64{g.Lat[v], g.Lon[v]} }
	sample := func(nodes []uint32, limit int) [][2]float64 {
		step := max(1, len(nodes)/limit)
		var out [][2]float64
		for i := 0; i < len(nodes); i += step {
			out = append(out, pt(nodes[i]))
		}
		return out
	}
	km := 0.0
	coords := make([][2]float64, len(path))
	for i, v := range path {
		coords[i] = pt(v)
		if i > 0 {
			km += shortcut.Haversine(g.Lat[path[i-1]], g.Lon[path[i-1]], g.Lat[v], g.Lon[v]) / 1000
		}
	}
	// Road backdrop: every node inside the box around path and search, thinned.
	minLat, maxLat, minLon, maxLon := 90.0, -90.0, 180.0, -180.0
	for _, v := range append(slices.Clone(path), settled...) {
		minLat, maxLat = min(minLat, g.Lat[v]), max(maxLat, g.Lat[v])
		minLon, maxLon = min(minLon, g.Lon[v]), max(maxLon, g.Lon[v])
	}
	var inBox []uint32
	for v := range g.NumNodes() {
		if g.Lat[v] >= minLat && g.Lat[v] <= maxLat && g.Lon[v] >= minLon && g.Lon[v] <= maxLon {
			inBox = append(inBox, uint32(v))
		}
	}
	out := map[string]any{
		"algo":         algo,
		"ms":           ms,
		"km":           km,
		"queryMs":      float64(took.Microseconds()) / 1000,
		"settledCount": len(settled),
		"path":         coords,
		"settled":      sample(settled, 40000),
		"roads":        sample(inBox, 80000),
	}
	b, err := json.Marshal(out)
	if err != nil {
		return err
	}
	return os.WriteFile(file, b, 0o644)
}
