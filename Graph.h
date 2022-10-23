#pragma once
#include <list>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <sstream>

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
    explicit Edge(int k) : start(nullptr), end(nullptr), key(k) {}
    Edge(const Edge&) = delete;
    Edge(Edge&&) = default;
    bool operator==(const Edge& other) const {
        // ONLY WORKS WITHIN THE SAME GRAPH
        return other.key == key;
    }
    Node& otherNode(const Node& n) {
        if (n == *start) return *end;
        return *start;
    }

};

struct Graph {
    std::vector<std::unique_ptr<Edge>> edges;
    std::vector<std::unique_ptr<Node>> nodes;
    int next_edge_key = 0;
    int next_node_key = 0;

    Graph() {}
    Graph(const Graph&) = delete;
    Graph(Graph&&) = default;
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
    std::pair<int, Edge&> addUnboundEdge() {
        int key = next_edge_key++;
        auto& edge = edges.emplace_back(std::make_unique<Edge>(key));
        return { key, *edge };
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

    static Vector outwardDirFrom(const GeomEdge&, const GeomNode&);
    bool compare(const GeomNode&, const GeomEdge&, const GeomEdge&) const;

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
    auto getEdges() const {
        std::vector<const GeomEdge*> ret;
        for (const auto& ge : geom_edges) {
            ret.push_back(ge.get());
        }
        return ret;
    }
    size_t numEdges() const {
        return geom_edges.size();
    }

    size_t numNodes() const {
        return geom_nodes.size();
    }

    auto& otherEdgesItem(const Node& n, const Edge& e) {
        if (n == *e.start) return getGeomEdge(e.key).end_edges_item;
        return getGeomEdge(e.key).start_edges_item;
    }
};

struct PolyEdge {
    using NodeT = Polygon;
    // The left and right polygons with respect to geom_edge's direction
    // are the PolyEdge's edge's start and end nodes respectively.
    Edge& edge;
    const GeomEdge& geom_edge; // the geom edge it is the dual edge of

    PolyEdge(Edge& e, const GeomEdge& ge);
};


struct Polygon {
    Node& node;
    std::string name;
    explicit Polygon(Node& n, std::string&& nm) : node(n), name(std::move(nm)) {}
};

struct PolyGraph {
    const GeomGraph& geom_graph;
    Graph g;
    std::vector<std::unique_ptr<Polygon>> polygons;
    std::vector<std::unique_ptr<PolyEdge>> poly_edges;

    const Polygon& getPolygon(int key) const {
        return *polygons[key];
    }
    Polygon& getPolygon(int key) {
        return *polygons[key];
    }
    const PolyEdge& getPolyEdge(int key) const {
        return *poly_edges[key];
    }
    PolyEdge& getPolyEdge(int key) {
        return *poly_edges[key];
    }
};

class PolygonBuilder {
private:
    struct PolygonBuildStep {
        Node* node;
        Node::EdgesListType::iterator edge_to_process;
    };
    const GeomGraph& geom_graph;
    Graph g;
    std::vector<std::unique_ptr<Polygon>> polygons;
    std::vector<std::unique_ptr<PolyEdge>> poly_edges{geom_graph.numEdges()};
    std::queue<PolygonBuildStep> steps;
    int name_counter = 0;

    static std::string generateName(int counter) {
        std::stringstream ss;
        do {
            ss << static_cast<char>('A' + (counter % 26));
            counter /= 26;
        } while (counter > 0);
        return ss.str();
    }

    static void addPolyEdgeToBackOfPolygon(Polygon& poly, PolyEdge* e) {
        poly.node.edges.emplace_back(&e->edge);
    }

    static void addPolyEdgeToFrontOfPolygon(Polygon& poly, PolyEdge* e) {
        poly.node.edges.emplace_front(&e->edge);
    }

    Polygon* geometricLeftPolygon(const GeomEdge& ge) const {
        if (poly_edges[ge.edge.key] == nullptr) return nullptr;
        auto* poly_node = poly_edges[ge.edge.key]->edge.start;
        if (poly_node == nullptr) return nullptr;
        return polygons[poly_node->key].get();
    }

    Polygon* geometricRightPolygon(const GeomEdge& ge) const {
        if (poly_edges[ge.edge.key] == nullptr) return nullptr;
        auto* poly_node = poly_edges[ge.edge.key]->edge.end;
        if (poly_node == nullptr) return nullptr;
        return polygons[poly_node->key].get();
    }

    Polygon* outwardLeftPolygon(const Node& n, const GeomEdge& ge) const {
        if (poly_edges[ge.edge.key] == nullptr) return nullptr;
        if (n == *ge.edge.start) return geometricLeftPolygon(ge);
        return geometricRightPolygon(ge);
    }

    Polygon* outwardRightPolygon(const Node& n, const GeomEdge& ge) const {
        if (poly_edges[ge.edge.key] == nullptr) return nullptr;
        if (n == *ge.edge.start) return geometricRightPolygon(ge);
        return geometricLeftPolygon(ge);
    }

    std::pair<int, Polygon&> addPolygon(std::string&& name) {
        auto n = g.addNode();
        auto& gn = polygons.emplace_back(std::make_unique<Polygon>(n.second, std::move(name)));
        return { n.first, *gn };
    }

    void addPolyEdge(const GeomEdge& ge) {
        auto& e = g.getEdge(ge.edge.key);
        poly_edges[ge.edge.key] = std::make_unique<PolyEdge>(e, ge);
    }

    void addGeometricLeftPolyToPolyEdge(Polygon* p, PolyEdge& pe) {
        pe.edge.start = &p->node;
    }

    void addGeometricRightPolyToPolyEdge(Polygon* p, PolyEdge& pe) {
        pe.edge.end = &p->node;
    }

    auto& saveOutwardRightPolyWithRespectToGeomEdge(const Node& n, const GeomEdge& ge,
        Polygon* outward_right_poly) {
        if (poly_edges[ge.edge.key] == nullptr) {
            addPolyEdge(ge);
        }
        auto& pe = poly_edges[ge.edge.key];
        if (n == *ge.edge.start) {
            addGeometricRightPolyToPolyEdge(outward_right_poly, *pe);
            return pe;
        }
        addGeometricLeftPolyToPolyEdge(outward_right_poly, *pe);
        return pe;
    }

    void enqueue(Node* node, Node::EdgesListType::iterator& edge_to_process) {
        steps.emplace(node, edge_to_process);
    }

    // Moves along the left spans starting from one edge.
    // Builds the right hand side polygon
    void buildFromOnePolygonEdge() {
        if (steps.empty()) {
            return;
        }
        auto& start_step = steps.front();
        auto step = start_step;
        auto* outward_right_poly =
            outwardRightPolygon(*step.node,
                geom_graph.getGeomEdge((*step.edge_to_process)->key));

        if (outward_right_poly == nullptr) {
            outward_right_poly = &addPolygon(generateName(name_counter++)).second;
        }
        else {
            steps.pop();
            return; // this edge's right side was already processed starting from elsewhere
        }
        do {
            auto* current_node = step.node;
            auto& current_edge = step.edge_to_process;
            auto* other_node = &(*current_edge)->otherNode(*current_node);
            auto& other_end_edges_it = const_cast<GeomGraph&>(geom_graph).otherEdgesItem(
                *current_node, **current_edge);

            const GeomEdge* ge = &geom_graph.getGeomEdge((*current_edge)->key);

            auto& pe = saveOutwardRightPolyWithRespectToGeomEdge(*current_node, *ge, outward_right_poly);
            addPolyEdgeToFrontOfPolygon(*outward_right_poly, pe.get());

            Node* geom_left_poly_node = pe->edge.start;
            Node* geom_right_poly_node = pe->edge.end;
            if (geom_left_poly_node == nullptr || geom_right_poly_node == nullptr) {
                // If the one poly is missing then enqueue the 
                // backwards direction which will make the other
                // side poly of it.
                enqueue(other_node, other_end_edges_it);
            }

            // Advance along the left span of the other node
            step = PolygonBuildStep{ other_node,
                    other_end_edges_it.circularAdvancedBy(1) };
        } while (step.node != start_step.node || step.edge_to_process != start_step.edge_to_process);
        steps.pop();

        // If we could not schedule anything that means we have found all the
        // polygons.
    }
public:
    PolygonBuilder(const GeomGraph& gg) : geom_graph(gg) {
        for (size_t i = 0; i < geom_graph.numEdges(); ++i) {
            g.addUnboundEdge();
        }
    }
    PolyGraph buildPolygons(const GeomNode& start_node) {
        auto it = start_node.node.edges.begin();
        enqueue(&start_node.node, it);
        while (!steps.empty()) {
            buildFromOnePolygonEdge();
        }
        return {geom_graph, std::move(g), std::move(polygons), std::move(poly_edges)};
    }

    PolyGraph buildPolygons() {
        return buildPolygons(geom_graph.getGeomNode(0));
    }
};
