struct DisjointsSets {
    std::vector<int> parent;
    std::vector<int> rank;
    int n;

    DisjointsSets(int n){
        this->n = n;

        parent.resize(n+1);
        rank.resize(n+1);

        for(int i = 0; i <= n; i++){
            rank[i] = 0;
            parent[i] = i;
        }
    }

    int find(int u){
        if(u != parent[u]) parent[u] = find(parent[u]); // Else if u is not the parent of itself, then u is not the \
        representative of his set. So we recursively call find on its parent
        return parent[u]; // if u is the parent of itself, then u is the representative of this set
    }

    void merge(int x, int y){
        // Find the representatives (ranks) for the set that includes x and y
        x = find(x);
        y = find(y);


        // If y’s rank is less than x’s rank 
        if(rank[x] > rank[y]) parent[y] = x; // Then move y under x
        else parent[x] = y; // Else move x under y

        if(rank[x] == rank[y]) rank[y]++; // if the ranks are equal, increment the result tree's rank by 1
    }
};
    