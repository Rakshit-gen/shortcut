// Package shortcut is a road routing engine over OpenStreetMap data.
package shortcut

import (
	"bufio"
	"encoding/binary"
	"errors"
	"fmt"
	"io"
	"os"
	"slices"
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

// NewGraph builds a CSR graph with n nodes from an unsorted edge list.
// Lat and Lon are left for the caller to set.
func NewGraph(n int, edges []Edge) *Graph { return csr(n, edges, nil) }

// csr builds the CSR arrays with a counting sort. If pos is not nil,
// pos[i] is set to the slot edges[i] landed in.
func csr(n int, edges []Edge, pos []uint32) *Graph {
	g := &Graph{
		FirstOut: make([]uint32, n+1),
		Head:     make([]uint32, len(edges)),
		Weight:   make([]uint32, len(edges)),
	}
	for _, e := range edges {
		g.FirstOut[e.From+1]++
	}
	for v := range n {
		g.FirstOut[v+1] += g.FirstOut[v]
	}
	next := slices.Clone(g.FirstOut[:n])
	for i, e := range edges {
		p := next[e.From]
		next[e.From]++
		g.Head[p], g.Weight[p] = e.To, e.W
		if pos != nil {
			pos[i] = p
		}
	}
	return g
}

func (g *Graph) NumNodes() int { return len(g.FirstOut) - 1 }
func (g *Graph) NumEdges() int { return len(g.Head) }

// Reverse returns the graph with every edge flipped.
func (g *Graph) Reverse() *Graph {
	edges := make([]Edge, 0, g.NumEdges())
	for u := range g.NumNodes() {
		for i := g.FirstOut[u]; i < g.FirstOut[u+1]; i++ {
			edges = append(edges, Edge{g.Head[i], uint32(u), g.Weight[i]})
		}
	}
	r := NewGraph(g.NumNodes(), edges)
	r.Lat, r.Lon = g.Lat, g.Lon
	return r
}

// Version 3: one SCC in k-d tree order (see Prepare), edge weights rounded up.
var graphMagic = [8]byte{'s', 'h', 'c', 't', 'g', 'r', 'f', '3'}

func (g *Graph) Save(path string) error {
	return writeBin(path, graphMagic, uint64(g.NumNodes()), uint64(g.NumEdges()),
		g.FirstOut, g.Head, g.Weight, g.Lat, g.Lon)
}

func Load(path string) (*Graph, error) {
	g := &Graph{}
	err := readBin(path, graphMagic, func(read func(...any)) {
		var n, m uint64
		read(&n, &m)
		g.FirstOut, g.Head, g.Weight = make([]uint32, n+1), make([]uint32, m), make([]uint32, m)
		g.Lat, g.Lon = make([]float64, n), make([]float64, n)
		read(g.FirstOut, g.Head, g.Weight, g.Lat, g.Lon)
	})
	return g, err
}

func writeBin(path string, vals ...any) error {
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	w := bufio.NewWriterSize(f, 1<<20)
	for _, v := range vals {
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

// readBin checks the file's magic, then hands fn a reader that keeps the first
// error. Sizes read before an error are zero, so fn's allocations stay small.
func readBin(path string, magic [8]byte, fn func(read func(...any))) error {
	f, err := os.Open(path)
	if err != nil {
		return err
	}
	defer f.Close()
	r := bufio.NewReaderSize(f, 1<<20)
	read := func(vals ...any) {
		for _, v := range vals {
			if err == nil {
				err = binary.Read(r, binary.LittleEndian, v)
			}
		}
	}
	var m [8]byte
	read(&m)
	if err == nil && m != magic {
		return fmt.Errorf("%s: wrong file type or an older format, rebuild it", path)
	}
	fn(read)
	if err != nil {
		return fmt.Errorf("%s: %w", path, err)
	}
	if _, err := r.ReadByte(); err != io.EOF {
		return errors.New(path + ": trailing data")
	}
	return nil
}
