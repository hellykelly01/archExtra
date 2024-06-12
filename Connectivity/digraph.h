#ifndef DIGRAPH_H
#define DIGRAPH_H

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <fstream>
#include <vector>
#include <queue>

#include "MinPrioityQueue.h"
#include "Stack.h"


struct pair_hash {
  size_t operator()(const std::pair<int, int>& p) const {
      return std::hash<int>()(p.first) ^ std::hash<int>()(p.second);
  }
};

template<typename T>

class DiGraph{
public:
  DiGraph():
    idCount(0)
  {}

  void InsertVertex(const T& v){
    int id;
    if(valToId.count(v) == 0){
      id = ++idCount;
      valToId[v] = id;
      idToVal[id] = v;
    } else {
      id = valToId[v];
    }
    outgoing.emplace(id, std::unordered_map<int, double>());
    incoming.emplace(id, std::unordered_set<int>());
  }

  void InsertEdge(const T& from, const T& to, double weight){
    if(valToId.count(from) == 0 || valToId.count(to) == 0){
      throw std::runtime_error("no such vertex while edge insert");
    }
    int idFrom = valToId[from];
    int idTo = valToId[to];
    outgoing[idFrom][idTo] = weight;
    incoming[idTo].insert(idFrom);
  }

  std::pair<bool, double> GetEdgeWeight(const T& from, const T& to){
    bool exist = false;
    double weight = -1;
    if(valToId.count(from) == 0 || valToId.count(to) == 0){
      throw std::runtime_error("no such vertexes");
    }
    int idFrom = valToId[from];
    int idTo = valToId[to];
    if(outgoing[idFrom].count(idTo) != 0){
      exist = true;
      weight = outgoing[idFrom][idTo];
    }
    return std::make_pair(exist, weight);
  }

  void RemoveEdge(const T& from, const T& to){
    if(valToId.count(from) == 0 || valToId.count(to) == 0){
      throw std::runtime_error("no such vertex while edge insert");
    }
    int idFrom = valToId[from];
    int idTo = valToId[to];
    outgoing[idFrom].erase(idTo);
    incoming[idTo].erase(idFrom);
  }

  void RemoveVertex(const T& v){
    if(valToId.count(v) == 0){
      throw std::runtime_error("no such vertex while vertex remove");
    }
    int id = valToId[v];
    for(const auto& [to, weight] : outgoing[id]){
      incoming[to].erace(id);
    }
    for(const auto& from : incoming[v]){
      outgoing[from].erace(id);
    }
    outgoing.erase(id);
    incoming.erase(id);
    valToId.erase(v);
    idToVal.erase(id);
  }

  void generateDOT(const std::string& filename){
    std::ofstream outfile (filename);
    outfile << "digraph {\n";
    outfile << "    node [fontname=\"Arial\"];\n";
    for(const auto& [value, id] : valToId){
      outfile << "    " << id << " [label=\"" << value << "\"];\n";
    }
    for(const int id : fakeIds){
      outfile << "    " << id << " [label=\"\"];\n";
    }
    for(const auto& [from, destinaton] : outgoing){
      for(const auto& [to, weight] : destinaton){
        outfile << "    " << from << " -> " << to << " [label=\"" << weight << "\"];\n";
      }
    }
    outfile << "}\n";
    outfile.close();
  }

  std::pair<std::vector<T>, double> Dijkstra(const T& from, const T& to){
    if(valToId.count(from) == 0 || valToId.count(to) == 0){
      throw std::runtime_error("no such vertex while edge insert");
    }
    int idFrom = valToId[from];
    int idTo = valToId[to];

    std::unordered_set<int> discovered;
    std::unordered_map<int, int> prevVertexes;
    MinPiorityQueue<int> pq;
    std::unordered_map<int, typename MinPiorityQueue<int>::Locator> pqLocator;
    double totalWeight = -1;

    for (const auto& [k, v] : outgoing){
      if(k == idFrom){
        pqLocator[k] = pq.InsertElement(k, 0);
      } else{
        pqLocator[k] = pq.InsertElement(k, MinPiorityQueue<int>::inf);
      }
    }

    while(pq.GetHeapSize() > 0){
      auto [vertex, weight] = pq.ExtractMin();
      if (weight == MinPiorityQueue<int>::inf){
        break;
      }
      discovered.insert(vertex);
      pqLocator.erase(vertex);
      if (vertex == idTo){
        totalWeight = weight;
        break;
      }
      for(const auto& [_to, _weight] : outgoing[vertex]){
        if(discovered.count(_to) == 0){
          double updatedWeight = weight + _weight;
          if(pq.GetWeight(pqLocator[_to].Get()) > updatedWeight){
            pq.DecreaseWeight(pqLocator[_to].Get(), updatedWeight);
            prevVertexes[_to] = vertex;
          }
        }
      }
    }

    std::vector<T> path;

    if(discovered.count(idTo) != 0){
      path = GetPath(prevVertexes, idFrom, idTo);
    }
    return std::make_pair(path, totalWeight);
  }

  std::pair<std::vector<T>, bool> BFS(const T& from, const T& to){
    if(valToId.count(from) == 0 || valToId.count(to) == 0){
      throw std::runtime_error("no such vertex while edge insert");
    }
    int idFrom = valToId[from];
    int idTo = valToId[to];

    std::unordered_set<int> discovered;
    std::queue<int> vertexQueue;
    std::unordered_map<int, int> prevVertexes;

    vertexQueue.push(idFrom);
    discovered.insert(idFrom);
    while (!vertexQueue.empty()){
      if(vertexQueue.front() == idTo){
        break;
      } else {
        for(const auto& [_to, _weight] : outgoing[vertexQueue.front()]){
          if(discovered.count(_to) == 0){
            vertexQueue.push(_to);
            discovered.insert(_to);
            prevVertexes[_to] = vertexQueue.front();
          }
        }
        vertexQueue.pop();
      }
    }

    std::vector<T> path;
    bool found = false;

    if(discovered.count(idTo) != 0){
      found = true;
      path = GetPath(prevVertexes, idFrom, idTo);
    }
    return std::make_pair(path, found);
  }

  std::pair<DiGraph, double> MaxFlow(const T& from, const T& to){
    if(valToId.count(from) == 0 || valToId.count(to) == 0){
      throw std::runtime_error("no such vertex while edge insert");
    }
    int idFrom = valToId[from];
    int idTo = valToId[to];

    std::unordered_set<std::pair<int, int>, pair_hash> fix;
    std::vector<std::pair<int, int>> backEdges;

    DiGraph residual = *this;
    for (const auto& [_from, destination] : outgoing){
      for (const auto& [_to, weight] : destination){
        if(residual.outgoing[_to].count(_from) != 0){
          auto needFix = std::make_pair(_to, _from);
          if(fix.count(std::make_pair(_from, _to)) == 0){
            fix.insert(needFix);
          } 
        } else {
          backEdges.push_back(std::make_pair(_to, _from));
        }
      }
    }

    for (const std::pair<int, int>& p : fix){
      int fakeId = ++residual.idCount;
      residual.fakeIds.insert(fakeId);
      double weight = residual.outgoing[p.first][p.second];
      residual.outgoing[p.first][p.second] = 0;

      residual.outgoing[p.first][fakeId] = weight;
      residual.incoming[fakeId].insert(p.first);
      residual.outgoing[fakeId][p.second] = weight;
      residual.incoming[p.second].insert(fakeId);

      residual.outgoing[fakeId][p.first] = 0;
      residual.incoming[p.first].insert(fakeId);
      residual.outgoing[p.second][fakeId] = 0;
      residual.incoming[fakeId].insert(p.second);


    }

    for (const std::pair<int, int>& p : backEdges){
      residual.outgoing[p.first][p.second] = 0;
      residual.incoming[p.second].insert(p.first);
    }

    double ans = 0;

    while (true){
      double delta = residual.BFSupdate(idFrom, idTo);
      if(delta > 0){
        ans += delta;
      } else {
        break;
      }
    }

    return std::make_pair(residual, ans);
  }

  double CountEdgeConnectivity(){
    double ans = -1;
    T firstVertex;
    bool first = true;
    for (const auto& [k, v] : valToId){
      if(first){
        first = false;
        firstVertex = k;
      } else{
        ans = ans == -1 ? MaxFlow(firstVertex, k).second : std::min(MaxFlow(firstVertex, k).second, ans);
      }
    }

    return ans;
  }

private:
  std::unordered_set<int> fakeIds;
  std::unordered_map<T, int> valToId;
  std::unordered_map<int, T> idToVal;
  std::unordered_map<int, std::unordered_map<int, double>> outgoing;
  std::unordered_map<int, std::unordered_set<int>> incoming;
  int idCount;

  std::vector<T> GetPath(std::unordered_map<int, int>& prevVertexes, int idFrom, int idTo){
    std::vector<T> path;
    Stack<T> revPath;
    int current = idTo;
    while(current != idFrom){
      revPath.push(idToVal[current]);
      current = prevVertexes[current];
    }
    path.push_back(idToVal[idFrom]);
    while (!revPath.empty()){
      path.push_back(revPath.top());
      revPath.pop();
    }
    return path;
  }

  double BFSupdate(int idFrom, int idTo){
    std::unordered_map<int, double> discovered;
    std::unordered_map<int, int> prevVertexes;
    std::queue<int> vertexQueue;

    vertexQueue.push(idFrom);
    discovered[idFrom] = std::numeric_limits<double>::max();
    while (!vertexQueue.empty()){
      if(vertexQueue.front() == idTo){
        break;
      } else {
        for(const auto& [_to, _weight] : outgoing[vertexQueue.front()]){
          if(discovered.count(_to) == 0 && _weight != 0){
            vertexQueue.push(_to);
            discovered[_to] = std::min(_weight, discovered[vertexQueue.front()]);
            prevVertexes[_to] = vertexQueue.front();
          }
        }
        vertexQueue.pop();
      }
    }

    double ans = -1;

    if(discovered.count(idTo) != 0){
      ans = discovered[idTo];
      int current = idTo;
      while(current != idFrom){
        outgoing[prevVertexes[current]][current] -= ans;
        outgoing[current][prevVertexes[current]] += ans;
        current = prevVertexes[current];
      }
    }

    return ans;

  }

};

#endif // DIGRAPH_H