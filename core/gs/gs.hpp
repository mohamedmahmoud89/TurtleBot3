#ifndef GRAPH_SEARCH_HPP
#define GRAPH_SEARCH_HPP

#include "common.hpp"
#include <queue>
#include <unordered_set>

using namespace std;

class GraphSearch{
public:
    bool search(const GraphNodeList& grid,const GraphNode& start, const GraphNode& goal,list<GraphNode>& path){
        if(start==goal)
            return true;
        queue<GraphNode>q;
        unordered_map<GraphNode,GraphNode,hashFunc>m;
        unordered_set<GraphNode,hashFunc>visited;
        q.push(start);
        u16 temp=0;
        while(!q.empty()&&temp++<100){
            GraphNode curr=q.front();
            visited.insert(curr);
            q.pop();
            auto it=grid.find(curr);
            for(auto& n:it->second){
                m[n]=curr;
                if(n==goal){
                    GraphNode temp=goal;
                    /*while(!(temp==start)){
                        path.push_front(m[temp]);
                        temp=m[temp];
                    }*/
                    path.push_front(start);
                    return true;
                }
                if(visited.find(n)==visited.end())
                    q.push(n);
            }
        }

        return false;
    };
};

#endif