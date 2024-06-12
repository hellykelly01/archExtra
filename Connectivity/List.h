#ifndef LIST_H
#define LIST_H

#include <stdexcept>
#include <iostream>
#include <vector>

// List для бедных

template <typename T>
class List{
public:
  
  struct Node{
    T value;
    Node* next = nullptr;
    Node* prev = nullptr;
  };
  
  List(){}

  List(const List& other){
    if(other.head){
      for(Node* node = other.head; node; node = node->next){
        pushBack(node->value);
      }
    }
  }
 
  List(List&& other){
    head = other.head;
    tail = other.tail;
    other.head = nullptr;
    other.tail = nullptr;
    _size = other._size;
  }

  List& operator=(const List& other){
    if(&other == this){
      return *this;
    }
    List tmp(other);
    std::swap(head, tmp.head);
    std::swap(tail, tmp.tail);
    std::swap(_size, tmp._size);
    return *this;
  }

  List& operator=(List&& other){
    if(&other == this){
      return *this;
    }
    List tmp(other);
    std::swap(head, tmp.head);
    std::swap(tail, tmp.tail);
    std::swap(_size, tmp._size);
    return *this;
  }

  void pushFront(const T& value){
    Node* newNode = new Node;
    newNode->value = value;
    if (head == nullptr) {
      head = tail = newNode;
    } else {
      head->prev = newNode;
      newNode->next = head;
      head = newNode;
    }
    ++_size;
  };

  void pushBack(const T& value){
    Node* newNode = new Node;
    newNode->value = value;
    if (head == nullptr) {
      head = tail = newNode;
    } else {
      tail->next = newNode;
      newNode->prev = tail;
      tail = newNode;
    }
    ++_size;
  }

  void popFront(){
    if (head == nullptr) {
        return;
    }
    Node* nodeToRemove = head;
    head = head->next;
    if (head != nullptr) {
      head->prev = nullptr;
    } else {
      tail = nullptr;
    }
    delete nodeToRemove;
    --_size;
  }

  void popBack(){
    if (tail == nullptr) {
        return;
    }
    Node* nodeToRemove = tail;
    tail = tail->prev;
    if (tail != nullptr) {
      tail->next = nullptr;
    } else {
      head = nullptr;
    }
    delete nodeToRemove;
    --_size;
  }

  void remove(Node* node){
    if(node == head){
      popFront();
    } else if (node == tail){
      popBack();
    } else {
      node->prev->next = node->next;
      node->next->prev = node->prev;
      delete node;
      --_size;
    }

  };

  T& front(){
    if(head == nullptr){
      throw std::runtime_error("front to empy list");
    }
    return head->value;
  }

  const T& front() const{
    if(head == nullptr){
      throw std::runtime_error("Front to empy list");
    }
    return head->value;
  }

  T& back(){
    if(tail == nullptr){
      throw std::runtime_error("back to empy list");
    }
    return tail->value;
  }

  const T& back() const{
    if(tail == nullptr){
      throw std::runtime_error("back to empy list");
    }
    return tail->value;
  }
  
  int size() const {
    return _size;
  }

  Node* GetHead(){
    return head;
  }

  std::vector<T> toVector(){
    std::vector<T> ans;
    ans.reserve(_size);
    for (Node* node = head; node; node = node->next){
      ans.push_back(node->value);
    }
    return ans;
  }

  template <typename V>
  friend std::ostream& operator<<(std::ostream& os, const List<V>&);

  ~List(){
    while(head){
      Node* newHead = head->next;
      delete head;
      head =  newHead;
    }
  }

private:
  Node* head = nullptr;
  Node* tail = nullptr;
  int _size = 0;
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const List<T>& list){
  for (auto node = list.head; node; node = node->next){
    os << node->value << ' ';
  }
  return os;
}

#endif