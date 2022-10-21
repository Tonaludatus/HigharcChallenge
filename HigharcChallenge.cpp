
#include <iostream>
#include <list>

#include "Constants.h"
#include "Point.h"
#include "Vector.h"

struct Edge;

struct Node {
    Point pos;
    std::list<Edge*> edges; // ordered by rotation of dir
};

struct Edge {
    Node& start;
    Node& end;
    Vector dir; // normalized

    Edge(Node& s, Node& e) : start(s), end(e), dir(Vector(s.pos, e.pos).normalized()) {}
};

int main()
{
    std::cout << "Hello World!\n";
}
