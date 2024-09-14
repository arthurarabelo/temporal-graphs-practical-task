#include "graph.hpp"

int main(){

    ulli num_v, num_e;
    ulli latestYear = 0;
    std::cin >> num_v >> num_e;

    Graph graph(num_v, num_e);

    ulli from, to, a, l, c;
    for(ulli i = 0; i < num_e; i++){
        std::cin >> from >> to >> a >> l >> c;
        if(a > latestYear) latestYear = a;
        graph.addEdge(from, to, a, l, c);
    }
    graph.setLatestYear(latestYear);

    // first problem
    graph.printDistances();
    std::cout << graph.firstYearForShortestPaths() << std::endl;
    
    // second problem
    std::cout << graph.earliestYearConnected() << std::endl;

    // third problem
    std::cout << graph.kruskalMST() << std::endl;

    return 0;
}