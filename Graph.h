#pragma once
#include <list>
#include <memory>
#include <optional>
#include <queue>
#include <string>

#include "CircularList.h"
#include "Constants.h"
#include "Point.h"
#include "Vector.h"

struct GeomNode;
struct Polygon;
struct PolyEdge;
struct Edge;

struct Node {
    using EdgesListType = CList<Edge*>;
    int key;
    EdgesListType edges; // ordered
    Node() : key(0) {}
    Node(int k) : key(k) {}
    Node(const Node&) = delete;
    Node(Node&&) = default;
    bool operator==(const Node& other) const {
        // ONLY WORKS WITHIN THE SAME GRAPH
        return other.key == key;
    }
};


struct Edge {
    int key;
    Node* start;
    Node* end;

    Edge() : start(nullptr), end(nullptr), key(0) {}
    Edge(Node& s, Node& e, int k) : start(&s), end(&e), key(k) {}
    Edge(const Edge&) = delete;
    Edge(Edge&&) = default;
    bool operator==(const Edge& other) const {
        // ONLY WORKS WITHIN THE SAME GRAPH
        return other.key == key;
    }
};

struct Graph {
    std::vector<std::unique_ptr<Edge>> edges;
    std::vector<std::unique_ptr<Node>> nodes;
    int next_edge_key = 0;
    int next_node_key = 0;
    std::pair<int, Node&> addNode() {
        int key = next_node_key++;
        auto& node = nodes.emplace_back(std::make_unique<Node>(key));
        return { key, *node };
    };
    std::pair<int, Edge&> addEdge(Node& s, Node& e) {
        int key = next_edge_key++;
        auto& edge = edges.emplace_back(std::make_unique<Edge>(s, e, key));
        return {key, *edge};
    }
    const Node& getNode(int key) const {
        return *nodes[key];
    }
    Node& getNode(int key) {
        return *nodes[key];
    }
    const Edge& getEdge(int key) const {
        return *edges[key];
    }
    Edge& getEdge(int key) {
        return *edges[key];
    }
};

// Edges ordered by rotation from (1, 0)
struct GeomNode {
    Node& node;
    Point pos;
    GeomNode(Node& n, const Point& p) : node(n), pos(p) {}
};

class GeomGraph;

struct GeomEdge {
    using NodeT = GeomNode;
    Edge& edge;
    Vector dir; // normalized
    Node::EdgesListType::iterator start_edges_item;
    Node::EdgesListType::iterator end_edges_item;
    GeomEdge(GeomGraph& g, Edge& e, GeomNode& start, GeomNode& end);
};


class GeomGraph {
private:
    friend struct GeomNode;
    friend struct GeomEdge;
    friend class GraphTest;
    Graph g;
    std::vector<std::unique_ptr<GeomEdge>> geom_edges;
    std::vector<std::unique_ptr<GeomNode>> geom_nodes;
    Node::EdgesListType::iterator insertEdge(GeomNode&, GeomEdge*);
    bool compare(const GeomNode&, const GeomEdge&, const GeomEdge&) const;

    Vector outwardDirFrom(const GeomEdge&, const GeomNode&) const;
public:
    std::pair<int, GeomNode&> addGeomNode(const Point& p) {
        auto n = g.addNode();
        auto& gn = geom_nodes.emplace_back(std::make_unique<GeomNode>(n.second, p));
        return { n.first, *gn };
    }
    std::pair<int, GeomEdge&> addGeomEdge(GeomNode& s, GeomNode&e) {
        auto edge = g.addEdge(s.node, e.node);
        auto& ge = geom_edges.emplace_back(std::make_unique<GeomEdge>(*this, edge.second, s, e));
        return { edge.first, *ge };
    }
    const GeomNode& getGeomNode(int key) const {
        return *geom_nodes[key];
    }
    GeomNode& getGeomNode(int key) {
        return *geom_nodes[key];
    }
    const GeomEdge& getGeomEdge(int key) const {
        return *geom_edges[key];
    }
    GeomEdge& getGeomEdge(int key) {
        return *geom_edges[key];
    }
};
//using PolyEdgeAdder = void(*)(Polygon&, PolyEdge*);
//
//struct PolyEdge : public Edge<Polygon, GeomEdge*> {
//    using NodeT = Polygon;
//    GeomEdge* geom_edge;
//
//    PolyEdge(Polygon& s, Polygon& e, PolyEdgeAdder sadder, PolyEdgeAdder eadder, GeomEdge* ge);
//};
//
//
//struct Polygon : public Node<std::string, PolyEdge> {
//    std::string& name;
//    explicit Polygon(const std::string& n) : Node(n), name(data) {}
//};
//
//struct PolygonBuildStep {
//    const GeomNode& next;
//    std::optional<const CList<GeomEdge*>::iterator> from_edge;
//    Polygon* outgoing_left_poly;
//    PolyEdgeAdder outgoing_left_edge_adder;
//    Polygon* outgoing_rigth_poly;
//    PolyEdgeAdder outgoing_right_edge_adder;
//};
//
//// ****************DEFINITIONS******************
//
//void addPolyEdgeToBack(Polygon& poly, PolyEdge* e) {
//    poly.edges.emplace_back(e);
//}
//
//void addPolyEdgeToFront(Polygon& poly, PolyEdge* e) {
//    poly.edges.emplace_front(e);
//}

//void buildPolygonEdges(std::list<Polygon> polys, std::queue<PolygonBuildStep> steps) {
//    if (steps.empty()) {
//        return;
//    }
//    auto& step = steps.front();
//    auto start_edge = step.from_edge.has_value() ? *(step.from_edge) : step.next.edges.begin();
//    auto span_start = start_edge;
//    auto span_end = span_start.circularAdvancedBy(1);
//    auto start_poly = step.outgoing_left_poly;
//    auto start_adder = step.outgoing_left_edge_adder;
//    auto left_poly = step.outgoing_left_poly;
//    auto left_adder = step.outgoing_left_edge_adder;
//    do {
//        
//    } while (span_end != start_edge);
//    steps.pop();
//}

Vector GeomGraph::outwardDirFrom(const GeomEdge& e, const GeomNode& n) const {
    if (n.node == *e.edge.start) return e.dir;
    return e.dir.flipped();
}

GeomEdge::GeomEdge(GeomGraph&g, Edge& e, GeomNode& start, GeomNode& end) 
    : edge(e), dir(Vector(start.pos, end.pos).normalized()),
    start_edges_item(g.insertEdge(start, this)),
    end_edges_item(g.insertEdge(end, this)) {}


//PolyEdge::PolyEdge(Polygon& s, Polygon& e, PolyEdgeAdder sadder, PolyEdgeAdder eadder, GeomEdge* ge) :
//    Edge(s, e, std::move(ge)) {
//    sadder(s, this);
//    eadder(e, this);
//}

bool GeomGraph::compare(const GeomNode&n, const GeomEdge& one, const GeomEdge& other) const {
    return ccwLessNormalized(outwardDirFrom(one, n), outwardDirFrom(other, n));
}

Node::EdgesListType::iterator GeomGraph::insertEdge(GeomNode& n, GeomEdge* e) {
    auto& edges_list = n.node.edges;
    auto it = edges_list.begin();
    while (it != edges_list.end() && compare(n, getGeomEdge((*it)->key), *e)) {
        ++it;
    }
    return edges_list.emplace(it, &(e->edge));
}

