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
    int position; //posição de inserção do node
    bool directed;
    bool weighted_edge;
    bool weighted_node;

    Node *first_node;
    Node *last_node;

public:
    //Constructor & Destructor
    Graph(int order, bool directed, bool weighted_edge, bool weighted_node);
    ~Graph();

    //Getters
    int getOrder();
    int getNumberEdges();
    bool getDirected();
    bool getWeightedEdge();
    bool getWeightedNode();
    Node *getFirstNode();
    Node *getLastNode();
    Node *getNode(int id);
    Node *getNodePosition(int position); //Pega um node apartir de sua posição de inserção
    Graph *getVertInduz();

    // Other methods
    void insertNode(int id);
    void insertEdge(int id, int target_id, float weight);
    void removeNode(int id);
    bool searchNode(int id);
    void cleanVisited(); // Define todos os nós como não visitados.

    // Funcionalidades
    void directTransitiveClosing(int id, ofstream &outFile);
    void indirectTransitiveClosing(int id, ofstream &outFile);
    float dijkstra(ofstream &outFile);
    float floydWarshall(ofstream &outFile);
    void agmPrim(Graph *subgrafo, ofstream &outFile);
    void agmKruskal(Graph *subgrafo, ofstream &outFile);
    void deepSearch(ofstream &outFile);
    void topologicalSorting();
    void printDeepSearch(vector<string> *corpo,ofstream &outFile);

    /* Para 2a parte do trabalho*/
    void greed();
    void greedRandom();
    void greedRactiveRandom();
    

private:
    //Auxiliar methods
    void deepPath(Node *node);
    void auxDeepSearch(Node *node, vector<int> *finG, vector<int> *retorno, ofstream &outFile,  vector<string> *corpo);
    void caminhoMinimo(list<int> &antecessor, ofstream &outFile);
    int **iniciaAnterioresFloyd(int **anteriores, int tam);
    int **iniciaDistanciaFloyd(int **anteriores, int tam);
    void saidaFloyd(int **pred, Node *noSource, Node *noTarget, ofstream &outFile);
    void saidaDijkstra(int antecessor[], int idSource, int idTarget, ofstream &outFile);
    bool thisIsCyclic();
    bool auxIsCyclic(int nodeId, bool isVisited[], int parentId);
    bool isCyclic();
    bool auxIsCyclicDirected(int nodeId, bool isVisited[], bool *isContainedRecusirve);
    bool isCyclicDirected();
    void topologicalSortUtil(Node *node, Edge *edge, stack<int> &Stack);
};

#endif // GRAPH_H_INCLUDED
