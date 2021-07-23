#include "Edge.h"
#include <iostream>

using namespace std;

/**************************************************************************************************
 * Defining the Edge's methods
**************************************************************************************************/

// Constructor
Edge::Edge(int target_id, int target_position){

    this->target_id = target_id;
    this->next_edge = nullptr;
    this->weight = 0;
    this->target_position = target_position;         //posição padrão do nó target, mudar durante a inserção da aresta

}

// Destructor
Edge::~Edge(){

    if (this->next_edge != nullptr){
        delete this->next_edge;
        this->next_edge = nullptr;
    }

}

// Getters
int Edge::getTargetId(){

    return this->target_id;

}
int Edge::getTargetPosition(){

    return this->target_position;

}


Edge* Edge::getNextEdge(){

    return this->next_edge;

}

float Edge::getWeight(){

    return this->weight;

}

// Setters
void Edge::setNextEdge(Edge* edge){

    this->next_edge = edge;

}

void Edge::setWeight(float weight){

    this->weight = weight;

}
