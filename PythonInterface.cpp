#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vector>
#include <iostream>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt
using namespace boost;
namespace py = pybind11;

struct location
{
  float y, x; // lat, long
};

typedef float cost;

template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  distance_heuristic(LocMap l, Vertex goal)
    : m_location(l), m_goal(goal) {}
  CostType operator()(Vertex u)
  {
    CostType dx = m_location[m_goal].x - m_location[u].x;
    CostType dy = m_location[m_goal].y - m_location[u].y;
    return ::sqrt(dx * dx + dy * dy);
  }
private:
  LocMap m_location;
  Vertex m_goal;
};

struct found_goal {}; 

template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if(u == m_goal)
      throw found_goal();
  }
private:
  Vertex m_goal;
};

typedef adjacency_list<listS, vecS, undirectedS, no_property,
                       property<edge_weight_t, cost> > mygraph_t;
typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
typedef mygraph_t::vertex_descriptor astar_vertex;
typedef mygraph_t::edge_descriptor edge_descriptor;
typedef mygraph_t::vertex_iterator vertex_iterator;
typedef std::pair<int, int> astar_edge;

struct Node {
  location loc;
  std::size_t id;
};

class AStar {
  std::vector<location> nodes;
  std::vector<astar_edge> edges;
  mygraph_t graph;
  WeightMap wmap;

public:
  AStar(std::vector<location> nodes, std::vector<astar_edge> edges) {
    this->nodes = nodes;
    this->edges = edges;
    mygraph_t graph(this->nodes.size());
    this->graph = graph;
    this->wmap = get(boost::edge_weight, this->graph);
    for (std::size_t j = 0; j < this->edges.size(); ++j) {
      edge_descriptor e; bool inserted;
      boost::tie(e, inserted) = add_edge(this->edges[j].first,
                                         this->edges[j].second, this->graph);
      this->wmap[e] = 1.0;
    }
  }

  std::list<astar_vertex> run(astar_vertex start, astar_vertex goal) {

    auto g = this->graph;

    std::vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
    std::vector<cost> d(num_vertices(g));

    try {
      astar_search
        (this->graph, start,
         distance_heuristic<mygraph_t, cost, location*>
         (&(this->nodes)[0], goal),
         predecessor_map(&p[0]).distance_map(&d[0]).
         visitor(astar_goal_visitor<astar_vertex>(goal)));


    } catch(found_goal fg) {
      std::list<astar_vertex> shortest_path;
      for(astar_vertex v = goal;; v = p[v]) {
        shortest_path.push_front(v);
        if(p[v] == v)
          break;
      }
      std::list<astar_vertex>::iterator spi = shortest_path.begin();
      return shortest_path;
    }

    return std::list<astar_vertex>();

  }
};


PYBIND11_MODULE(astar_cpp, m) {
  py::class_<AStar>(m, "AStar")
    .def(py::init<std::vector<location>, std::vector<astar_edge>>())
    .def("run", &AStar::run);
  py::class_<location>(m, "location")
    .def(py::init<int, int>())
    .def_readonly("x", &location::x)
    .def_readonly("y", &location::y);
}