package shortcut

import (
	"context"
	"math"
	"os"
	"runtime"

	"github.com/paulmach/osm"
	"github.com/paulmach/osm/osmpbf"
)

// Car speeds in km/h by highway type. Ways with other highway values are skipped.
var speeds = map[string]float64{
	"motorway": 100, "motorway_link": 60,
	"trunk": 80, "trunk_link": 50,
	"primary": 60, "primary_link": 40,
	"secondary": 50, "secondary_link": 40,
	"tertiary": 40, "tertiary_link": 30,
	"unclassified": 30, "residential": 25,
	"living_street": 10, "service": 15,
}

// wayRule decides whether cars can use a way, at what speed, and in which directions.
func wayRule(t osm.Tags) (kmh float64, fwd, bwd bool) {
	kmh, ok := speeds[t.Find("highway")]
	if !ok {
		return 0, false, false
	}
	for _, k := range []string{"access", "motor_vehicle", "motorcar"} {
		if v := t.Find(k); v == "no" || v == "private" {
			return 0, false, false
		}
	}
	oneway := t.Find("oneway")
	switch {
	case oneway == "-1":
		return kmh, false, true
	case oneway == "yes" || oneway == "true" || oneway == "1":
		return kmh, true, false
	case oneway == "no":
		return kmh, true, true
	case t.Find("highway") == "motorway" || t.Find("junction") == "roundabout":
		return kmh, true, false
	}
	return kmh, true, true
}

type way struct {
	refs     []int64
	kmh      float64
	fwd, bwd bool
}

// ImportPBF reads car-routable roads from an .osm.pbf file in two passes:
// ways first (to learn which nodes matter), then those nodes' coordinates.
func ImportPBF(path string) (*Graph, error) {
	var ways []way
	need := map[int64]int32{}
	err := scan(path, false, func(o osm.Object) {
		w := o.(*osm.Way)
		kmh, fwd, bwd := wayRule(w.Tags)
		if !fwd && !bwd || len(w.Nodes) < 2 {
			return
		}
		refs := make([]int64, len(w.Nodes))
		for i, n := range w.Nodes {
			refs[i] = int64(n.ID)
			need[refs[i]] = -1
		}
		ways = append(ways, way{refs, kmh, fwd, bwd})
	})
	if err != nil {
		return nil, err
	}

	var lat, lon []float64
	err = scan(path, true, func(o osm.Object) {
		n := o.(*osm.Node)
		if id, ok := need[int64(n.ID)]; ok && id < 0 {
			need[int64(n.ID)] = int32(len(lat))
			lat, lon = append(lat, n.Lat), append(lon, n.Lon)
		}
	})
	if err != nil {
		return nil, err
	}

	var edges []Edge
	for _, w := range ways {
		for i := 1; i < len(w.refs); i++ {
			a, b := need[w.refs[i-1]], need[w.refs[i]]
			if a < 0 || b < 0 || a == b { // node outside the extract, or repeated node
				continue
			}
			m := haversine(lat[a], lon[a], lat[b], lon[b])
			ms := uint32(math.Max(1, math.Round(m/(w.kmh/3.6)*1000)))
			if w.fwd {
				edges = append(edges, Edge{uint32(a), uint32(b), ms})
			}
			if w.bwd {
				edges = append(edges, Edge{uint32(b), uint32(a), ms})
			}
		}
	}
	return NewGraph(lat, lon, edges).Prepare(), nil
}

func scan(path string, nodes bool, fn func(osm.Object)) error {
	f, err := os.Open(path)
	if err != nil {
		return err
	}
	defer f.Close()
	s := osmpbf.New(context.Background(), f, runtime.GOMAXPROCS(0))
	defer s.Close()
	s.SkipRelations = true
	s.SkipNodes = !nodes
	s.SkipWays = nodes
	for s.Scan() {
		fn(s.Object())
	}
	return s.Err()
}

// haversine returns the great-circle distance in meters.
func haversine(lat1, lon1, lat2, lon2 float64) float64 {
	const r = 6371000.0
	rad := math.Pi / 180
	dLat, dLon := (lat2-lat1)*rad, (lon2-lon1)*rad
	a := math.Sin(dLat/2)*math.Sin(dLat/2) +
		math.Cos(lat1*rad)*math.Cos(lat2*rad)*math.Sin(dLon/2)*math.Sin(dLon/2)
	return 2 * r * math.Asin(math.Sqrt(a))
}
