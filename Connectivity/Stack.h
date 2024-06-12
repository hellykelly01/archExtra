#ifndef STACK_H
#define STACK_H

#include "List.h"

template <typename T>
class Stack{
public:
  void push(const T& value){
    elements.pushBack(value);
  } 
  void pop(){
    elements.popBack();
  }
  T& top(){
    return elements.back();
  }
  const T& top() const{
    return elements.back();
  }
  bool empty(){
    return (elements.size() == 0);
  }
private:
  List<T> elements;
};

#endif