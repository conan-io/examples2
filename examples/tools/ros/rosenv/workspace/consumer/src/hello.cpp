#include <iostream>
#include <string>

#include "hello.h"

#include "Box2D/Box2D.h"
#include <boost/json.hpp>

using namespace boost::json;


void hello() {
    #ifdef NDEBUG
    std::cout << "Hello World Release! " << b2_maxPolygonVertices << std::endl;
    #else
    std::cout << "Hello World Debug! " << b2_maxPolygonVertices << std::endl;
    #endif

    object obj;
    obj[ "pi" ] = 3.141;
    obj[ "happy" ] = true;
    obj[ "name" ] = "Boost";
    obj[ "nothing" ] = nullptr;
    obj[ "answer" ].emplace_object()["everything"] = 42;
    obj[ "list" ] = { 1, 0, 2 };
    obj[ "object" ] = { {"currency", "USD"}, {"value", 42.99} };
    std::string s = serialize(obj);
    std::cout << s << std::endl;
}
