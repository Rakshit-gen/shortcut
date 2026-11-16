package shortcut

import "math"

// Prepare keeps only the largest strongly connected component, so every pair
// of nodes is reachable, and renumbers nodes so that ids form an implicit
// k-d tree: the node at the middle of any id range [lo, hi) splits that range
// on axis depth%3. Nearest uses that layout, and spatially close nodes get
// close ids, which keeps graph searches cache friendly.
func (g *Graph) Prepare() *Graph {
	keep := largestSCC(g)
	var order []uint32
	for v, k := range keep {
		if k {
			order = append(order, uint32(v))
		}
	}
	pts := make([][3]float64, g.NumNodes())
	for _, v := range order {
		pts[v] = toXYZ(g.Lat[v], g.Lon[v])
	}
	kdOrder(pts, order, 0)
	return g.permute(order)
}

// permute builds the subgraph on order, where order[newID] = oldID.
// Edges to nodes not in order are dropped.
func (g *Graph) permute(order []uint32) *Graph {
	newID := make([]uint32, g.NumNodes())
	for i := range newID {
		newID[i] = Inf
	}
	for nv, v := range order {
		newID[v] = uint32(nv)
	}
	lat, lon := make([]float64, len(order)), make([]float64, len(order))
	edges := make([]Edge, 0, g.NumEdges())
	for nv, v := range order {
		lat[nv], lon[nv] = g.Lat[v], g.Lon[v]
		for i := g.FirstOut[v]; i < g.FirstOut[v+1]; i++ {
			if w := newID[g.Head[i]]; w != Inf {
				edges = append(edges, Edge{uint32(nv), w, g.Weight[i]})
			}
		}
	}
	return NewGraph(lat, lon, edges)
}

// largestSCC runs Tarjan's algorithm with an explicit call stack (road graphs
// have paths millions of nodes deep) and marks the nodes of the largest SCC.
func largestSCC(g *Graph) []bool {
	n := g.NumNodes()
	index := make([]uint32, n) // 0 = unvisited, else discovery order + 1
	low := make([]uint32, n)
	comp := make([]uint32, n)
	onStack := make([]bool, n)
	var stack []uint32
	var sizes []int
	type frame struct{ v, edge uint32 }
	var call []frame
	next := uint32(1)

	visit := func(v uint32) {
		index[v], low[v] = next, next
		next++
		stack = append(stack, v)
		onStack[v] = true
		call = append(call, frame{v, g.FirstOut[v]})
	}
	for s := range n {
		if index[s] != 0 {
			continue
		}
		visit(uint32(s))
		for len(call) > 0 {
			f := &call[len(call)-1]
			v := f.v
			if f.edge < g.FirstOut[v+1] {
				w := g.Head[f.edge]
				f.edge++
				if index[w] == 0 {
					visit(w)
				} else if onStack[w] {
					low[v] = min(low[v], index[w])
				}
				continue
			}
			call = call[:len(call)-1]
			if len(call) > 0 {
				p := call[len(call)-1].v
				low[p] = min(low[p], low[v])
			}
			if low[v] == index[v] {
				id, size := uint32(len(sizes)), 0
				for {
					w := stack[len(stack)-1]
					stack = stack[:len(stack)-1]
					onStack[w] = false
					comp[w] = id
					size++
					if w == v {
						break
					}
				}
				sizes = append(sizes, size)
			}
		}
	}
	best := 0
	for c, s := range sizes {
		if s > sizes[best] {
			best = c
		}
	}
	keep := make([]bool, n)
	for v := range keep {
		keep[v] = n > 0 && comp[v] == uint32(best)
	}
	return keep
}

// toXYZ maps a coordinate onto the unit sphere. Straight-line distance there
// grows with great-circle distance, so Euclidean nearest is true nearest.
func toXYZ(lat, lon float64) [3]float64 {
	rad := math.Pi / 180
	la, lo := lat*rad, lon*rad
	return [3]float64{math.Cos(la) * math.Cos(lo), math.Cos(la) * math.Sin(lo), math.Sin(la)}
}

func kdOrder(pts [][3]float64, ids []uint32, depth int) {
	for len(ids) > 1 {
		mid := len(ids) / 2
		nthElement(pts, ids, mid, depth%3)
		kdOrder(pts, ids[:mid], depth+1)
		ids = ids[mid+1:] // loop on the right half instead of recursing
		depth++
	}
}

// nthElement partially sorts ids so ids[k] has the k-th smallest coordinate on
// axis, with smaller-or-equal ones before it and larger-or-equal after.
func nthElement(pts [][3]float64, ids []uint32, k, axis int) {
	lo, hi := 0, len(ids)-1
	for lo < hi {
		pivot := pts[ids[lo+(hi-lo)/2]][axis]
		i, j := lo, hi
		for i <= j {
			for pts[ids[i]][axis] < pivot {
				i++
			}
			for pts[ids[j]][axis] > pivot {
				j--
			}
			if i <= j {
				ids[i], ids[j] = ids[j], ids[i]
				i++
				j--
			}
		}
		switch {
		case k <= j:
			hi = j
		case k >= i:
			lo = i
		default:
			return
		}
	}
}

// Nearest returns the node closest to (lat, lon). It needs a graph that went
// through Prepare, since it reads the k-d tree from the node numbering.
func (g *Graph) Nearest(lat, lon float64) uint32 {
	q := toXYZ(lat, lon)
	best, bestD := uint32(0), math.Inf(1)
	var search func(lo, hi, depth int)
	search = func(lo, hi, depth int) {
		if lo >= hi {
			return
		}
		mid := lo + (hi-lo)/2
		p := toXYZ(g.Lat[mid], g.Lon[mid])
		dx, dy, dz := p[0]-q[0], p[1]-q[1], p[2]-q[2]
		if d := dx*dx + dy*dy + dz*dz; d < bestD {
			best, bestD = uint32(mid), d
		}
		diff := q[depth%3] - p[depth%3]
		if diff < 0 {
			search(lo, mid, depth+1)
			if diff*diff < bestD {
				search(mid+1, hi, depth+1)
			}
		} else {
			search(mid+1, hi, depth+1)
			if diff*diff < bestD {
				search(lo, mid, depth+1)
			}
		}
	}
	search(0, g.NumNodes(), 0)
	return best
}
