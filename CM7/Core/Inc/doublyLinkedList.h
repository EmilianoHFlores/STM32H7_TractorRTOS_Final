#ifndef __DOUBLYLINKEDLIST_H
#define __DOUBLYLINKEDLIST_H

/* header file contents go here */


#include <stdlib.h>
#include <stdio.h>


struct Node {
    double data;
    struct Node* next;
    struct Node* prev;
};

struct NodeCord {
    double x, y;
    struct NodeCord* next;
    struct NodeCord* prev;
};

struct doubleLinkedListCord {
    int size;
    struct NodeCord* head;
    struct NodeCord* tail;
};

struct doubleLinkedList {
    int size;
    int maxSize;
    double currSum;
    double mean;
    struct Node* head;
    struct Node* tail;
};

void DBLL_init(struct doubleLinkedList* list, int n);
void c_DBLL_init(struct doubleLinkedListCord* list);
void pop_front(struct doubleLinkedList* list);
void c_pop_front(struct doubleLinkedListCord* list);
void push_back(struct doubleLinkedList* list, double data);
void c_push_back(struct doubleLinkedListCord* list, double x, double y);
void traverse(struct doubleLinkedList* list);
void c_traverse(struct doubleLinkedListCord* list);

#endif /* __DOUBLYLINKEDLIST_H */
