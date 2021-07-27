/**************************************************************************************************
 * Implementation of the TAD Graph
**************************************************************************************************/

#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED
#include "Node.h"
#include <fstream>
#include <vector>
#include <stack>
#include <list>

using namespace std;

class Graph
{

    //Atributes
private:
    int order;
    int number_edges;

    int position;                       //posição de inserção do node

    bool directed;
    bool weighted_edge;
    bool weighted_node;
    Node *first_node;
    Node *last_node;

public:
    //Constructor
    Graph(int order, bool directed, bool weighted_edge, bool weighted_node);
    //Destructor
    ~Graph();
    //Getters
    int getOrder();
    int getNumberEdges();
    bool getDirected();
    bool getWeightedEdge();
    bool getWeightedNode();
    Node *getFirstNode();
    Node *getLastNode();
    //Other methods
    void insertNode(int id);
    void insertEdge(int id, int target_id, float weight);
    void removeNode(int id);
    bool searchNode(int id);
    Node *getNode(int id);
    Node *getNodePosition(int position);            //Pega um node apartir de sua posição de inserção
    void printGraph();
    void printGraph2();
    void printGraph_Dot_Directed();
    void printGraph_Dot_Not_Directed();
    void deepPath(Node *node);
    void directTransitiveClosing(int id);
    void indirectTransitiveClosing(int id);
    bool findEdge(int vet[], int k, int v);
    void deepSearch();
    void auxDeepSearch(Node *node, int vet[], int cont, vector<int> *ret);
    void cleanVisited(); // Define todos os nós como não visitados.

    void topologicalSortUtil(Node *node, Edge *edge, stack<int> &Stack);
    bool thisIsCyclic();
    bool isCyclic();
    bool auxIsCyclic(int nodeId, bool isVisited[], int parentId);
    bool isCyclicDirected();
    bool auxIsCyclicDirected(int nodeId, bool isVisited[], bool *isContainedRecusirve);

    //methods phase1
    void topologicalSorting();
    Graph *getVertInduz();
    void agmKruskal(Graph *subgrafo);
    Graph *agmPrim();
    float floydMarshall();
    float dijkstra();

    //methods phase1
    float greed();
    float greedRandom();
    float greedRactiveRandom();

private:
    //Auxiliar methods
    int **iniciaAnterioresFloyd(int **anteriores, int tam);
    int **iniciaDistanciaFloyd(int **anteriores, int tam);
    void imprimeFloyd(list<int>&antecessor);
    void imprimeDijkstra(int antecessor[], int idSource, int idTarget);
    
    
};

#endif // GRAPH_H_INCLUDED
