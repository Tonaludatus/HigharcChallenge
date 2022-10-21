#pragma once
#include <list>
#include <string>

#include "Constants.h"
#include "Point.h"
#include "Vector.h"

struct GeomNode;
struct Polygon;

template <typename Data, typename EdgeT, typename EdgeAssoc = EdgeT*>
struct Node {
    Data data;
    std::list<EdgeAssoc> edges; // ordered
    Node(const Data& d) : data(d) {};
    Node(const Node&) = delete;
    Node(Node&&) = delete;
    bool operator==(const Node& other) const {
        return &other == this;
    }
    static void insertEdge(typename EdgeT::NodeT&, EdgeAssoc&&);
};


template <typename NodeT, typename Assoc>
struct Edge {
    NodeT& start;
    NodeT& end;
    Assoc assoc;

    Edge(NodeT& s, NodeT& e, Assoc&& a) : start(s), end(e), assoc(std::move(a)) {
// This requires runtime type inference to pass 'this' as the correct type.
// Javascript will be able to do it...
//        NodeT::insertEdge(start, this);
//        NodeT::insertEdge(end, this);
    }
};

struct GeomEdge : public Edge<GeomNode, Vector> {
    using NodeT = GeomNode;
    Vector& dir; // normalized
    Vector outward_dir_from(const GeomNode& n) const;

    GeomEdge(GeomNode& s, GeomNode& e);

    // This should be a virtual method of the Node
    static bool compare(const GeomNode& n, const GeomEdge* one, const GeomEdge* other);
};

struct PolyEdge : public Edge<Polygon, GeomEdge*> {
    using NodeT = Polygon;
    struct IndexedPolyEdge {
        int index;
        PolyEdge* edge;
    };

    PolyEdge(Polygon& s, Polygon& e, int sindex, int eindex, GeomEdge* ge);

    // This should be a virtual method of the Node
    static bool compare(const Polygon& n, const IndexedPolyEdge& one, const IndexedPolyEdge& other);
};

// Edges ordered by rotation from (1, 0)
struct GeomNode : public Node<Point, GeomEdge> {
    Point& pos;
    explicit GeomNode(const Point& p) : Node(p), pos(data) {}
};

struct Polygon : public Node<std::string, PolyEdge, PolyEdge::IndexedPolyEdge> {
    std::string& name;
    explicit Polygon(const std::string& n) : Node(n), name(data) {}
};

// ****************DEFINITIONS******************

Vector GeomEdge::outward_dir_from(const GeomNode& n) const {
    if (n == start) return dir;
    return dir.flipped();
}

GeomEdge::GeomEdge(GeomNode& s, GeomNode& e) : Edge(s, e, Vector(s.pos, e.pos).normalized()), dir(assoc) {
    GeomNode::insertEdge(start, this);
    GeomNode::insertEdge(end, this);
}

bool GeomEdge::compare(const GeomNode& n, const GeomEdge* one, const GeomEdge* other) {
    return ccw_less_normalized(one->outward_dir_from(n), other->outward_dir_from(n));
}

PolyEdge::PolyEdge(Polygon& s, Polygon& e, int sindex, int eindex, GeomEdge* ge) : Edge(s, e, std::move(ge)) {
    Polygon::insertEdge(start, {sindex, this});
    Polygon::insertEdge(end, {eindex, this});
}

bool PolyEdge::compare(const Polygon& n, const IndexedPolyEdge& one, const IndexedPolyEdge& other) {
    return one.index < other.index;
}

template <typename Data, typename EdgeT, typename EdgeAssoc>
void Node<Data, EdgeT, EdgeAssoc>::insertEdge(typename EdgeT::NodeT& n, EdgeAssoc&& e) {
    auto it = n.edges.begin();
    while (it != n.edges.end() && EdgeT::compare(n, *it, e)) {
        ++it;
    }
    n.edges.insert(it, std::move(e));
}

