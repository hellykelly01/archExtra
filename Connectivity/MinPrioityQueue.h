#ifndef MINPRIORITYQUEUE
#define MINPRIORITYQUEUE

#include <vector>
#include <iostream>
#include <limits>

template<typename T>

class MinPiorityQueue{
public:

  static const double inf;

  struct Locator {
  public:
    explicit Locator(int* ptr){
      pos = ptr;
    }

    Locator(){
      pos = nullptr;
    }

    int Get(){
      return *pos;
    }
  private:
    int* pos = nullptr;
  };
  
  struct Element{
    double weight;
    T value;
    int* pos = nullptr;
  };

  MinPiorityQueue():
    heap_size(0) 
  {
    elements.push_back(Element());
  }

  MinPiorityQueue(const MinPiorityQueue& other):
    MinPiorityQueue()
  {
    for (int i = 1; i <= other.heap_size; ++i){
      InsertElement(other.elements[i].value, other.elements[i].weight);
    }
  }

  MinPiorityQueue& operator=(const MinPiorityQueue& other){
    if(&other == this){
      return *this;
    }
    MinPiorityQueue tmp(other);
    std::swap(tmp.elements, elements);
    std::swap(tmp.heap_size, heap_size);
    return *this;
  }

  MinPiorityQueue(MinPiorityQueue&& other){
    elements = std::move(other.elements);
    heap_size = other.heap_size;
    other.heap_size = 0;
    other.elements.push_back(Element());
  }

  MinPiorityQueue& operator=(MinPiorityQueue&& other){
    if(&other == this){
      return *this;
    }
    MinPiorityQueue tmp(other);
    std::swap(tmp.elements, elements);
    std::swap(tmp.heap_size, heap_size);
    return *this;
  }

  double GetWeight(int pos) const{
    if (pos > heap_size){
      throw std::runtime_error("pos greater then heap_size");
    }
    return elements[pos].weight;
  }

  void DecreaseWeight(int pos, double weight){
    if (pos > heap_size){
      throw std::runtime_error("pos greater then heap_size");
    }
    if (weight > elements[pos].weight){
      throw std::runtime_error("new weight is greater than current");
    }
    elements[pos].weight = weight;
    while (pos > 1 && elements[Parent(pos)].weight > elements[pos].weight){
      std::swap(*elements[pos].pos, *elements[Parent(pos)].pos);
      std::swap(elements[pos], elements[Parent(pos)]);
      pos = Parent(pos);
    }
  }

  Locator InsertElement(const T& element, double weight){
    ++heap_size;
    if(heap_size < elements.size()){
      elements[heap_size].weight = inf;
      elements[heap_size].value = element;
      elements[heap_size].pos = new int(heap_size);
    } else {
      elements.push_back({inf, element, new int(heap_size)});
    }
    Locator l(elements[heap_size].pos);
    DecreaseWeight(heap_size, weight);
    return l;
  }

  std::pair<T, double> ExtractMin(){
    if(heap_size < 1){
      throw std::runtime_error("can't extract min, heap is empty");
    }
    std::swap(*elements[1].pos, *elements[heap_size].pos);
    std::swap(elements[1], elements[heap_size]);
    delete elements[heap_size].pos;
    --heap_size;
    MinHeapify(1);
    return std::make_pair(std::move(elements[heap_size + 1].value), elements[heap_size + 1].weight);
  }

  int GetHeapSize() const{
    return heap_size;
  }

  std::vector<T> ValuesToVector() const{
    std::vector<T> ans;
    for (int i = 1; i <= heap_size; ++i){
      ans.push_back(elements[i].value);
    }
    return ans;
  }

  std::vector<double> WeightsToVector() const{
    std::vector<double> ans;
    for (int i = 1; i <= heap_size; ++i){
      ans.push_back(elements[i].weight);
    }
    return ans;
  }

  ~MinPiorityQueue(){
    for(int i = 1; i <= heap_size; ++i){
      delete elements[i].pos;
    }
  }

private:

  int Parent(int i){
    return i / 2;
  }

  int Left(int i){
    return 2 * i;
  }

  int Right(int i){
    return 2 * i + 1;
  }

  void MinHeapify(int pos){
    int l = Left(pos);
    int r = Right(pos);
    int smallest = -1;
    if(l <= heap_size && elements[l].weight < elements[pos].weight){
      smallest = l;
    } else {
      smallest = pos;
    }
    if(r <= heap_size && elements[r].weight < elements[smallest].weight){
      smallest = r;
    }
    if (smallest != pos){
      std::swap(*elements[pos].pos, *elements[smallest].pos);
      std::swap(elements[pos], elements[smallest]);
      MinHeapify(smallest);
    }
  }

  std::vector<Element> elements;
  int heap_size;
};

template<typename T>
const double MinPiorityQueue<T>::inf = std::numeric_limits<double>::max();

#endif