// Command shortcut imports OSM road data and answers routing queries.
package main

import (
	"errors"
	"flag"
	"fmt"
	"math/rand/v2"
	"os"
	"slices"
	"strconv"
	"strings"
	"time"

	"github.com/Rakshit-gen/shortcut"
)

const usage = `usage:
  shortcut import <in.osm.pbf> <out.graph>
  shortcut route  <graph> <lat,lon> <lat,lon>
  shortcut bench  <graph> [-n queries] [-seed n]`

func main() {
	if len(os.Args) < 3 {
		die(usage)
	}
	var err error
	switch os.Args[1] {
	case "import":
		err = importCmd(os.Args[2:])
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
	if len(args) != 3 {
		return errors.New(usage)
	}
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
	s, t := g.Nearest(lat1, lon1), g.Nearest(lat2, lon2)
	d := shortcut.NewDijkstra(g)
	start := time.Now()
	ms, path := d.Route(s, t)
	took := time.Since(start)
	if ms == shortcut.Inf {
		return fmt.Errorf("no route between nodes %d and %d", s, t)
	}
	fmt.Printf("time %s, %d nodes on path, %d settled, query %s\n",
		time.Duration(ms)*time.Millisecond, len(path), d.Settled, took.Round(time.Microsecond))
	return nil
}

func benchCmd(args []string) error {
	fs := flag.NewFlagSet("bench", flag.ExitOnError)
	n := fs.Int("n", 1000, "number of random queries")
	seed := fs.Uint64("seed", 1, "random seed")
	fs.Parse(args[1:])
	g, err := shortcut.Load(args[0])
	if err != nil {
		return err
	}
	r := rand.New(rand.NewPCG(*seed, 0))
	d := shortcut.NewDijkstra(g)
	times := make([]time.Duration, *n)
	settled, unreachable := 0, 0
	for i := range times {
		s, t := uint32(r.IntN(g.NumNodes())), uint32(r.IntN(g.NumNodes()))
		start := time.Now()
		ms, _ := d.Route(s, t)
		times[i] = time.Since(start)
		settled += d.Settled
		if ms == shortcut.Inf {
			unreachable++
		}
	}
	slices.Sort(times)
	pct := func(p float64) time.Duration { return times[int(p*float64(len(times)-1))] }
	fmt.Printf("dijkstra: %d queries, p50 %s, p99 %s, avg settled %d, unreachable %d\n",
		*n, pct(0.5), pct(0.99), settled / *n, unreachable)
	return nil
}
