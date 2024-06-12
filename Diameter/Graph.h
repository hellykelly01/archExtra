#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <limits>

template<typename T>

class Graph{
public:
  Graph() = default;

  void InsertVertex(const T& v){
    neigh.emplace(v, std::unordered_set<T>());
  }

  void InsertEdge(const T& v1, const T& v2){
    if(neigh.count(v1) == 0 || neigh.count(v2) == 0){
      throw std::runtime_error("no such vertex while edge insert");
    }
    neigh[v1].insert(v2);
    neigh[v2].insert(v1);
  }

  void RemoveEdge(const T& v1, const T& v2){
    if(neigh.count(v1) == 0 || neigh.count(v2) == 0){
      throw std::runtime_error("no such edge while edge remove");
    }
    neigh[v1].erase(v2);
    neigh[v2].erase(v1);
  }

  void RemoveVertex(const T& v){
    if(neigh.count(v) == 0){
      throw std::runtime_error("no such vertex while vertex remove");
    }
    for(const auto& n : neigh[v]){
      neigh[n].erase(v);
    }
    neigh.erase(v);
  }

  int acyclicDiameter(){
    if (neigh.size() == 0){
      return -1;
    }
    return BFSfarthest(BFSfarthest(neigh.begin()->first).first).second;
  }

  int generalDiameter(){
    int ans = -1;
    std::vector<std::vector<int>> dist = FW();
    for (int i =0; i < dist.size(); ++i){
      for(int j = 0; j < dist.size(); ++j){
        if(dist[i][j] > ans){
          ans = dist[i][j];
        }
      }
    }

    return ans;
  }

private:
  
  std::unordered_map<T, std::unordered_set<T>> neigh;

  std::pair<T, int> BFSfarthest(const T& from){
    if(neigh.count(from) == 0){
      throw std::runtime_error("no such vertex");
    }

    std::queue<T> vertexQueue;
    std::unordered_map<T, int> discovered;

    vertexQueue.push(from);
    discovered[from] = 0;

    T last;
    while (!vertexQueue.empty()){
      last = vertexQueue.front();
      for(const auto& v : neigh[vertexQueue.front()]){
        if(discovered.count(v) == 0){
          vertexQueue.push(v);
          discovered[v] = discovered[vertexQueue.front()] + 1;
        }
      }
      vertexQueue.pop();
      
    }

    return std::make_pair(last, discovered[last]);
  }

  std::vector<std::vector<int>> FW(){
    std::vector<std::vector<int>> result;
    int id = -1;
    std::unordered_map<T, int> valToId;

    for(const auto& [k, v] : neigh){
      valToId[k] = ++id;
    }

    result.reserve(id + 1);
    for(int i = 0; i < id + 1; ++i){
      result.push_back(std::vector<int>(id + 1, -1));
    }

    for(const auto& [k, v] : neigh){
      for(const auto& k2 : v){
        result[valToId[k]][valToId[k2]] = 1;
      }
      result[valToId[k]][valToId[k]] = 0;
    }

    for (int k = 0; k < result.size(); ++k){
      for (int i = 0; i < result.size(); ++i){
        for (int j = 0; j < result.size(); ++j){
          int potential;
          if (result[i][k] == -1 || result[k][j] == -1){
            continue;
          } else {
            potential = result[i][k] + result[k][j];
          }
          if (result[i][j] == -1 || result[i][j] > potential){
            result[i][j] = potential;
          }    
        }
      }
    }   
    return result;
  }

};

#endif // GRAPH_H