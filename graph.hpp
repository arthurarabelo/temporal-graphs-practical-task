#include <vector>
#include <limits>
#include <iostream>
#include <algorithm>
#include <queue>
#include <tuple>
#include "disjointsSets.hpp"

constexpr unsigned long long int INFINITY = std::numeric_limits<unsigned long long int>::max();

typedef unsigned long long int ulli;

typedef std::pair<ulli, ulli> integer_pair;

class Vertex{
    public:
        Vertex() : visited(false) {} 
        Vertex(ulli i){
            this->visited = false;
        }
        
        bool visited;
        std::vector<std::tuple<ulli, ulli, ulli, ulli>> neighbors; // {v(destine vertex), a, l, c}
};

struct Edge{
    ulli a, l, c, from, to;

    Edge() : a(0), l(0), c(0), from(0), to(0) {}
    Edge(ulli from, ulli to){
        this->from = from;
        this->to = to;
    }
    Edge(ulli from, ulli to, ulli a, ulli l, ulli c){
        this->a = a;
        this->l = l;
        this->c = c;
        this->from = from;
        this->to = to;
    }

    bool operator<(const Edge& other) const {
        return this->c < other.c;
    }

    bool operator==(const Edge& other) const {
        return ((from == other.from && to == other.to) || (from == other.to && to == other.from));
    }
};

class Graph{
    public:
        Graph(ulli v, ulli e){
            this->years.resize(v);
            this->visited.resize(v, false);
            this->vertices.resize(v);
            this->edges.resize(e);
            this->distances.resize(v, INFINITY);
        }

        // insert edge in the edges vector and in the neighbors vector of each vertex of the edge
        void addEdge(ulli from, ulli to, ulli a, ulli l, ulli c){
            Edge e(from, to, a, l, c);
            this->edges.push_back(e);
            std::vector<ulli> neighbors_weights = {a, l, c};
            this->vertices[from-1].neighbors.push_back(std::make_tuple(to, a, l, c));
            this->vertices[to-1].neighbors.push_back(std::make_tuple(from, a, l, c));
        }

        // find the vertex with the minimum distance from the source
        ulli minDistanceVertex(){
            ulli minimum = INFINITY;
            ulli index;

            for(ulli k = 0; k < this->vertices.size(); k++){
                if(this->vertices[k].visited == false && this->distances[k] <= minimum){
                    minimum = this->distances[k];
                    index = k;
                }
            }

            return index;
        }

        // return the first year where shortest paths are mutually achievable
        ulli firstYearForShortestPaths(){
            ulli max_year = *std::max_element(this->years.begin(), this->years.end()); // find the bigger year within the years vector
            return max_year;
        }

        // find shortest paths from a source vertex to all the other vertices
        void dijkstra(ulli src){
            std::priority_queue<integer_pair, std::vector<integer_pair>, std::greater<integer_pair>> priority_queue;
            priority_queue.push(std::make_pair(src, 0));
            this->distances[src - 1] = 0; // distance from source to source is 0
            this->years[src] = 0;

            while(!priority_queue.empty()){
                ulli u = priority_queue.top().first; // u = vertices[u-1]
                priority_queue.pop();

                std::vector<std::tuple<ulli, ulli, ulli, ulli>>::iterator i;
                // iterate through neigbors of u
                for (i = this->vertices[u - 1].neighbors.begin(); i != this->vertices[u - 1].neighbors.end(); ++i) {
                    std::tuple<ulli, ulli, ulli, ulli> element = (*i); // element = {v, a, l, c}
                    ulli v = std::get<0>(element); // v = vertices[v-1]
                    ulli weight = std::get<2>(element); // weight = edge{u,v}.l
                    ulli year = std::get<1>(element); // year = edge{u,v}.a
                    if (this->distances[v-1] > this->distances[u-1] + weight) {
                        this->distances[v-1] = this->distances[u-1] + weight;
                        priority_queue.push(std::make_pair(v, this->distances[v-1]));
                        this->years[v-1] = year;
                    }
                }
            }
        }

        // print shortest paths to all vertices
        void printDistances(){
            this->dijkstra(1);
            for(ulli distance : distances){
                std::cout << distance << std::endl;
            }
        }

        // breadth-first search to check if graph is connected
        bool isConnectedThisYearBFS(ulli year){
            std::queue<ulli> q;
            ulli aux = 1;
            this->visited.assign(this->vertices.size(), false);

            this->visited[0] = true;
            q.push(1);

            while(!q.empty()){
                ulli currentNode = q.front();
                q.pop();

                std::vector<std::tuple<ulli, ulli, ulli, ulli>>::iterator i;
                for(i = this->vertices[currentNode - 1].neighbors.begin(); i != this->vertices[currentNode - 1].neighbors.end(); ++i){
                    std::tuple<ulli, ulli, ulli, ulli> element = (*i);
                    ulli current_year = std::get<1>(element); // current_year = edge{u,v}.a
                    ulli v = std::get<0>(element); // v = vertices[v-1]
                    if(current_year <= year && !visited[v-1]){
                        visited[v - 1] = true;
                        q.push(v);
                        aux++;
                    }
                }
            }

            if(aux == this->vertices.size()) return true; // if all vertices are achievable(visited) then graph is connected
            return false;
        }

        // binary search to seek the first year where all the nodes are mutually achievable
        ulli earliestYearConnected(){
            ulli lowestYearPossible = 1;
            ulli maxYear = this->latestYear;
            ulli earliestYear = -1;

            while(lowestYearPossible <= maxYear){
                ulli m = lowestYearPossible + ((maxYear - lowestYearPossible) / 2);

                if(this->isConnectedThisYearBFS(m)){
                    earliestYear = m;
                    maxYear = m - 1;
                } 
                else {
                    lowestYearPossible = m + 1;
                }
            }

            return earliestYear;
        }

        // kruskal algorithm to find the minimum spanning tree weight of the graph
        ulli kruskalMST(){
            ulli mst_weight = 0;

            // Sort all edges 
            std::sort(this->edges.begin(), this->edges.end());

            DisjointsSets ds(this->vertices.size());

            for(Edge edge : this->edges){
                ulli u = edge.from;
                ulli v = edge.to;

                ulli set_u = ds.find(u);
                ulli set_v = ds.find(v);

                // Take this edge in MST if it does 
                // not forms a cycle
                if(set_u != set_v){
                    mst_weight += edge.c;
                    ds.merge(set_u, set_v);
                }
            }

            return mst_weight;
        }

        void setLatestYear(ulli year){
            this->latestYear = year;
        }

    private:
        std::vector<bool> visited;
        std::vector<Edge> edges;
        std::vector<ulli> years;
        std::vector<Vertex> vertices;
        std::vector<ulli> distances;
        ulli latestYear;
};