#ifndef GRAPH_SEARCH_HPP
#define GRAPH_SEARCH_HPP

#include "common.hpp"
#include <queue>
#include <unordered_set>
#include <iostream>

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
        u16 temptemp=0;
        while(!q.empty()&&temptemp++<100){
            GraphNode curr=q.front();
            visited.insert(curr);
            q.pop();
            auto it=grid.find(curr);
            for(auto& n:it->second){
                if(visited.find(n)==visited.end()){
                    m[n]=curr;
                    if(n==goal){
                        GraphNode temp=goal;
                        while(!(temp==start)&&temptemp++<100){
                            path.push_front(temp);
                            temp=m[temp];
                            cout<<"temp.x= "<<temp.x<<endl;
                            cout<<"temp.y= "<<temp.y<<endl;
                        }
                        path.push_front(start);
                        /*for(auto&i:m){
                            cout<<"key.x= "<<i.first.x<<endl;
                            cout<<"key.y= "<<i.first.y<<endl;
                            cout<<"val.x= "<<i.second.x<<endl;
                            cout<<"val.y= "<<i.second.y<<endl;
                        }*/
                        return true;
                    }
                    q.push(n);
                }
            }
        }

        return false;
    };
};

#endif