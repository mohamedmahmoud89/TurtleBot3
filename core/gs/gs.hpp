#ifndef GRAPH_SEARCH_HPP
#define GRAPH_SEARCH_HPP

#include "common.hpp"
#include <queue>

using namespace std;

class GraphSearch{
public:
    bool search(const GraphNodeList& grid,const GraphNode& start, const GraphNode& goal,list<GraphNode>& path){
        if(start==goal)
            return true;
        queue<GraphNode>q;
        unordered_map<GraphNode,GraphNode,hashFunc>m;
        q.push(start);

        while(!q.empty()){
            GraphNode curr=q.front();
            q.pop();
            auto it=grid.find(curr);
            for(auto& n:it->second){
                m[n]=curr;
                if(n==goal){
                    GraphNode temp=goal;
                    while(!(goal==start)){
                        path.push_front(m[temp]);
                        temp=m[temp];
                    }
                    path.push_front(start);
                    return true;
                }
                q.push(n);
            }
        }

        return false;
    };
};

#endif