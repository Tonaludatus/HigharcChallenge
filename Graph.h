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

    auto& otherEdgesItem(const Node& n, const Edge& e) {
        if (n == *e.start) return getGeomEdge(e.key).end_edges_item;
        return getGeomEdge(e.key).start_edges_item;
    }
};

struct PolyEdge {
    using NodeT = Polygon;
    Edge& edge;
    // left and right are with respect to geom_edge's direction
    Polygon& left;
    Polygon& right;
    const GeomEdge* geom_edge; // the geom edge it is the dual edge of

    PolyEdge(Edge&, Polygon& l, Polygon& r, const GeomEdge* ge);
};


struct Polygon {
    Node& node;
    std::string name;
    explicit Polygon(Node& n, std::string&& nm) : node(n), name(std::move(nm)) {}
};

struct PolyGraph {
    Graph g;
    const GeomGraph& geom_graph;
    std::vector<std::unique_ptr<Polygon>> polygons;
    std::vector<std::unique_ptr<PolyEdge>> poly_edges;

    const Polygon& getPolygone(int key) const {
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
    struct PolyEdgeGeomEdgeAssociation {
        bool queued_or_done = false;
        PolyEdge* poly_edge = nullptr; // when not nullptr the associated edge is processed, that is, done.
    };
    struct PolygonBuildStep {
        Node* node;
        Node::EdgesListType::iterator edge_to_process;
    };
    Graph g;
    const GeomGraph& geom_graph;
    std::vector<std::unique_ptr<Polygon>> polygons;
    std::vector<std::unique_ptr<PolyEdge>> poly_edges;
    std::queue<PolygonBuildStep> steps;
    std::vector<PolyEdgeGeomEdgeAssociation> polyedges_of_geom_edges{geom_graph.numEdges()};
    int name_counter = 0;

    static Polygon& geomLeftPolyFromOutwardPolys(const Node& n, const GeomEdge& ge,
        Polygon& outward_left_poly, Polygon& outward_right_poly) {
        if (n == *ge.edge.start) return outward_left_poly;
        return outward_right_poly;

    }

    static Polygon& geomRightPolyFromOutwardPolys(const Node& n, const GeomEdge& ge,
        Polygon& outward_left_poly, Polygon& outward_right_poly) {
        if (n == *ge.edge.start) return outward_right_poly;
        return outward_left_poly;
    }

    static void addPolyEdgeToBackOfPolygon(Polygon& poly, PolyEdge* e) {
        poly.node.edges.emplace_back(&e->edge);
    }

    static void addPolyEdgeToFrontOfPolygon(Polygon& poly, PolyEdge* e) {
        poly.node.edges.emplace_front(&e->edge);
    }

    static std::string generateName(int counter) {
        std::stringstream ss;
        do {
            ss << static_cast<char>('A' + (counter % 26));
            counter /= 26;
        } while (counter > 0);
        return ss.str();
    }

    Polygon* outwardLeftPolygon(const Node& n, const GeomEdge& ge) const {
        PolyEdge* poly_edge = polyedges_of_geom_edges[ge.edge.key].poly_edge;
        if (poly_edge == nullptr) return nullptr;
        if (n == *ge.edge.start) return &(poly_edge->left);
        return &(poly_edge->right);
    }

    Polygon* outwardRightPolygon(const Node& n, const GeomEdge& ge) const {
        PolyEdge* poly_edge = polyedges_of_geom_edges[ge.edge.key].poly_edge;
        if (poly_edge == nullptr) return nullptr;
        if (n == *ge.edge.start) return &(poly_edge->right);
        return &(poly_edge->left);
    }

    std::pair<int, Polygon&> addPolygon(std::string&& name) {
        auto n = g.addNode();
        auto& gn = polygons.emplace_back(std::make_unique<Polygon>(n.second, std::move(name)));
        return { n.first, *gn };
    }

    std::pair<int, PolyEdge&> addPolyEdge(Polygon& left , Polygon& right, const GeomEdge* ge) {
        auto edge = g.addEdge(left .node, right.node);
        auto& pe = poly_edges.emplace_back(std::make_unique<PolyEdge>(edge.second, left , right, ge));
        polyedges_of_geom_edges[ge->edge.key] = { true, pe.get() };
        return { edge.first, *pe };
    }

    void enqueue(Node* node, Node::EdgesListType::iterator& edge_to_process) {
        steps.emplace(node, edge_to_process);
        polyedges_of_geom_edges[(*edge_to_process)->key].queued_or_done = true;
    }

    // Moves along the spans starting from one edge.
    // Queues unprocessed edges along the way.
    void buildFromOnePolygonEdge() {
        if (steps.empty()) {
            return;
        }
        auto& start_step = steps.front();
        auto step = start_step;
        do {
            auto* current_node = step.node;
            auto& current_edge = step.edge_to_process;
            auto* other_node = &(*current_edge)->otherNode(*current_node);
            auto& other_end_edges_it = const_cast<GeomGraph&>(geom_graph).otherEdgesItem(
                *current_node, **current_edge);
            if (polyedges_of_geom_edges[(*current_edge)->key].poly_edge == nullptr) {
                // New PolyEdge must be created for this geom edge
                auto span_end_left = current_edge.circularAdvancedBy(1);
                auto span_end_right = current_edge.circularAdvancedBy(-1);

                Polygon* span_start_outward_left_poly = outwardRightPolygon(*current_node, geom_graph.getGeomEdge((*span_end_left)->key));
                if (span_start_outward_left_poly == nullptr) {
                    // Try if the other end has it
                    span_start_outward_left_poly = outwardLeftPolygon(*other_node,
                        geom_graph.getGeomEdge((*other_end_edges_it.circularAdvancedBy(-1))->key));
                }
                if (span_start_outward_left_poly == nullptr) {
                    span_start_outward_left_poly = &addPolygon(generateName(name_counter++)).second;
                }

                auto* span_start_outward_right_poly = outwardLeftPolygon(*current_node, geom_graph.getGeomEdge((*span_end_right)->key));

                if (span_start_outward_right_poly == nullptr) {
                    // Try if the other end has it
                    span_start_outward_right_poly = outwardRightPolygon(*other_node,
                        geom_graph.getGeomEdge((*other_end_edges_it.circularAdvancedBy(1))->key));
                }
                if (span_start_outward_right_poly == nullptr) {
                    span_start_outward_right_poly = &addPolygon(generateName(name_counter++)).second;
                }

                const GeomEdge* ge = &geom_graph.getGeomEdge((*current_edge)->key);

                Polygon& geom_left_poly =
                    geomLeftPolyFromOutwardPolys(*current_node, *ge, *span_start_outward_left_poly, *span_start_outward_right_poly);
                Polygon& geom_right_poly =
                    geomRightPolyFromOutwardPolys(*current_node, *ge, *span_start_outward_left_poly, *span_start_outward_right_poly);
                auto pe = addPolyEdge(geom_left_poly, geom_right_poly, ge);
                addPolyEdgeToBackOfPolygon(*span_start_outward_left_poly, &pe.second);
                addPolyEdgeToFrontOfPolygon(*span_start_outward_right_poly, &pe.second);
            }

            // Advance along the left span of the other node
            step = PolygonBuildStep{ other_node,
                    other_end_edges_it.circularAdvancedBy(1) };
        } while (step.node != start_step.node);
        steps.pop();
        // Queue the only unprocessed edge of one of the visited
        // nodes.
        step = start_step;
        do {
            auto* current_node = step.node;
            auto& current_edge = step.edge_to_process;

            int num_unprocessed = 0;
            auto candidate = current_node->edges.end();
            for (auto it = current_node->edges.begin(); it != current_node->edges.end(); ++it) {
                if (!polyedges_of_geom_edges[(*it)->key].queued_or_done) {
                    ++num_unprocessed;
                    if (num_unprocessed > 1) break;
                    candidate = it;
                }
            }
            if (num_unprocessed == 1) {
                enqueue(current_node, candidate);
                return;
            }

            auto* other_node = &(*current_edge)->otherNode(*current_node);
            auto& other_end_edges_it = const_cast<GeomGraph&>(geom_graph).otherEdgesItem(
                *current_node, **current_edge);

            // Advance along the left span of the other node
            step = PolygonBuildStep{ other_node,
                    other_end_edges_it.circularAdvancedBy(1) };
        } while (step.node != start_step.node);
        // If no node with only one unprocessed edge was found,
        // enqueue the first righthand unprocessed edge from among the visited
        // nodes.
        step = start_step;
        do {
            auto* current_node = step.node;
            auto& current_edge = step.edge_to_process;

            auto* other_node = &(*current_edge)->otherNode(*current_node);
            auto& other_end_edges_it = const_cast<GeomGraph&>(geom_graph).otherEdgesItem(
                *current_node, **current_edge);

            auto candidate = other_node->edges.end();
            for (auto it = other_end_edges_it.circularAdvancedBy(-1);
                 it != other_end_edges_it;
                 it = it.circularAdvancedBy(-1)) {
                if (!polyedges_of_geom_edges[(*it)->key].queued_or_done) {
                    candidate = it;
                    enqueue(other_node, candidate);
                    return;
                }
            }

            // Advance along the left span of the other node
            step = PolygonBuildStep{ other_node,
                    other_end_edges_it.circularAdvancedBy(1) };
        } while (step.node != start_step.node);
        // If we could not schedule anything that means we have found all the
        // polygons.
    }
public:
    PolygonBuilder(const GeomGraph& gg) : geom_graph(gg) {}
    PolyGraph buildPolygons(const GeomNode& start_node) {
        auto it = start_node.node.edges.begin();
        enqueue(&start_node.node, it);
        while (!steps.empty()) {
            buildFromOnePolygonEdge();
        }
        return {std::move(g), geom_graph, std::move(polygons), std::move(poly_edges)};
    }

    PolyGraph buildPolygons() {
        return buildPolygons(geom_graph.getGeomNode(0));
    }
};
