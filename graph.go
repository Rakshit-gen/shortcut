// Package shortcut is a road routing engine over OpenStreetMap data.
package shortcut

import (
	"bufio"
	"encoding/binary"
	"errors"
	"io"
	"os"
)

// Graph is a directed road graph in compressed sparse row form.
// The out-edges of node v are Head[FirstOut[v]:FirstOut[v+1]].
type Graph struct {
	FirstOut []uint32 // len N+1
	Head     []uint32
	Weight   []uint32 // travel time in milliseconds
	Lat, Lon []float64
}

type Edge struct{ From, To, W uint32 }

// NewGraph builds a CSR graph from an unsorted edge list with a counting sort.
func NewGraph(lat, lon []float64, edges []Edge) *Graph {
	n := len(lat)
	g := &Graph{
		FirstOut: make([]uint32, n+1),
		Head:     make([]uint32, len(edges)),
		Weight:   make([]uint32, len(edges)),
		Lat:      lat,
		Lon:      lon,
	}
	for _, e := range edges {
		g.FirstOut[e.From+1]++
	}
	for v := 0; v < n; v++ {
		g.FirstOut[v+1] += g.FirstOut[v]
	}
	next := make([]uint32, n)
	copy(next, g.FirstOut[:n])
	for _, e := range edges {
		i := next[e.From]
		next[e.From]++
		g.Head[i], g.Weight[i] = e.To, e.W
	}
	return g
}

func (g *Graph) NumNodes() int { return len(g.FirstOut) - 1 }
func (g *Graph) NumEdges() int { return len(g.Head) }

var magic = [8]byte{'s', 'h', 'c', 't', 'g', 'r', 'f', '1'}

func (g *Graph) Save(path string) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	w := bufio.NewWriter(f)
	hdr := []any{magic, uint64(g.NumNodes()), uint64(g.NumEdges()),
		g.FirstOut, g.Head, g.Weight, g.Lat, g.Lon}
	for _, v := range hdr {
		if err := binary.Write(w, binary.LittleEndian, v); err != nil {
			f.Close()
			return err
		}
	}
	if err := w.Flush(); err != nil {
		f.Close()
		return err
	}
	return f.Close()
}

func Load(path string) (*Graph, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	r := bufio.NewReader(f)
	var m [8]byte
	var n, e uint64
	for _, v := range []any{&m, &n, &e} {
		if err := binary.Read(r, binary.LittleEndian, v); err != nil {
			return nil, err
		}
	}
	if m != magic {
		return nil, errors.New("not a shortcut graph file")
	}
	g := &Graph{
		FirstOut: make([]uint32, n+1),
		Head:     make([]uint32, e),
		Weight:   make([]uint32, e),
		Lat:      make([]float64, n),
		Lon:      make([]float64, n),
	}
	for _, v := range []any{g.FirstOut, g.Head, g.Weight, g.Lat, g.Lon} {
		if err := binary.Read(r, binary.LittleEndian, v); err != nil {
			return nil, err
		}
	}
	if _, err := r.ReadByte(); err != io.EOF {
		return nil, errors.New("trailing data in graph file")
	}
	return g, nil
}

// Nearest returns the node closest to (lat, lon). Linear scan for now.
func (g *Graph) Nearest(lat, lon float64) uint32 {
	best, bestD := uint32(0), -1.0
	for v := range g.Lat {
		d := haversine(lat, lon, g.Lat[v], g.Lon[v])
		if bestD < 0 || d < bestD {
			best, bestD = uint32(v), d
		}
	}
	return best
}
