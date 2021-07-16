#include <iostream>
#include <fstream>
#include <stack>
#include <queue>
#include <list>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <algorithm>
#include <set>
#include <vector>
#include <climits>
#include <math.h>
#include <float.h>

#include "Graph.h"
#include "Node.h"
#include "Edge.h"

#define INFINITO 9999;

using namespace std;

/**************************************************************************************************
 * Defining the Graph's methods
**************************************************************************************************/

// Constructor
Graph::Graph(int order, bool directed, bool weighted_edge, bool weighted_node)
{
    this->order = order;
    this->directed = directed;
    this->weighted_edge = weighted_edge;
    this->weighted_node = weighted_node;
    this->first_node = this->last_node = nullptr;
    this->number_edges = 0;
    for (int i = 0; i < order; i++)
    {
        this->insertNode(i + 1);
    }
}

// Destructor
Graph::~Graph()
{
    Node *next_node = this->first_node;

    while (next_node != nullptr)
    {
        next_node->removeAllEdges();
        Node *aux_node = next_node->getNextNode();
        delete next_node;
        next_node = aux_node;
    }
}

// Getters
int Graph::getOrder()
{
    return this->order;
}
int Graph::getNumberEdges()
{
    return this->number_edges;
}
//Function that verifies if the graph is directed
bool Graph::getDirected()
{
    return this->directed;
}
//Function that verifies if the graph is weighted at the edges
bool Graph::getWeightedEdge()
{
    return this->weighted_edge;
}

//Function that verifies if the graph is weighted at the nodes
bool Graph::getWeightedNode()
{
    return this->weighted_node;
}

Node *Graph::getFirstNode()
{
    return this->first_node;
}

Node *Graph::getLastNode()
{
    return this->last_node;
}

// Other methods
/*
    The outdegree attribute of nodes is used as a counter for the number of edges in the graph.
    This allows the correct updating of the numbers of edges in the graph being directed or not.
*/
void Graph::insertNode(int id)
{
    //confere se o grafo tem nodes
    if (first_node != nullptr)
    {
        //caso tenha, cria um novo node, aponta o ultimo pro novo e o novo de torna o ultimo
        Node *novo_node = new Node(id);
        last_node->setNextNode(novo_node);
        last_node = novo_node;
    }
    else
    {
        //caso nao tenha, cria um novo node e ele se torna o ultimo e o primeiro
        Node *novo_node = new Node(id);
        first_node = novo_node;
        last_node = novo_node;
    }
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    //cria um ponteiro para o node desejado e o um auxiliar para o node alvo da aresta
    Node *node = getNode(id);
    Node *aux = getNode(target_id);

    //confere se os nodes existem
    if (node != nullptr && aux != nullptr)
    {

        //confere se a aresta já existe
        if (!node->searchEdge(target_id))
        {
            //caso o node exista mas a aresta nao, insere a aresta
            node->insertEdge(target_id, weight);

            this->number_edges++;

            // se o grafo for nao-direcionado e nao houver aresta de volta
            if (this->directed == 0 && !aux->searchEdge(id))
            {
                //insere a aresta de volta
                aux->insertEdge(id, weight);
                // this->number_edges++;           ***** ESTÁ DOBRANDO O NUMERO DE ARESTAS
            }
        }
    }
}

void Graph::removeNode(int id)
{
    //cria um ponteiro para o node desejado
    Node *node = getNode(id);

    //retorna caso nao exista o node desejado
    if (node == nullptr)
        return;
    else if (node == first_node) //se o node eh o primeiro, apenas faz o proximo ser o primeiro
        first_node = node->getNextNode();
    else
    {
        //caso seja um node do meio ou o ultimo, cria um ponteiro auxiliar
        Node *aux = first_node;

        //encontra o node anterior ao node desejado
        while (aux->getNextNode() != node)
            aux = aux->getNextNode();

        //se o no desejado for o ultimo, atualiza o ultimo para o anterior
        if (node == last_node)
            last_node = aux;

        //seta o proximo de anterior para o proximo do desejado
        aux->setNextNode(node->getNextNode());
    }

    //deleta o node desejado
    delete node;
}

bool Graph::searchNode(int id)
{
    //cria um ponteiro para o node desejado
    Node *node = getNode(id);

    //retorna falso caso nao encontre e verdadeiro caso encontre
    if (node == nullptr)
        return false;
    else
        return true;
}

Node *Graph::getNode(int id)
{
    //cria ponteiro para percorrer a lista de nodes
    Node *node = first_node;

    //encontra o node com o id desejado
    while (node != nullptr)
    {
        if (node->getId() == id)
            break;

        node = node->getNextNode();
    }

    //retorna o node ou null caso nao encontre
    return node;
}

void Graph::printGraph()
{
    cout << "\nImprimindo grafo\n"
         << "Num Edge: " << this->number_edges << endl
         << "Num Edge: " << this->last_node << endl
         << "Num Edge: " << this->number_edges << endl;

    cout << "Ordem: " << this->getOrder() << endl;

    Node *node = first_node;
    while (node != nullptr)
    {
        cout << node->getId() << ": ";
        Edge *edge = node->getFirstEdge();
        while (edge != nullptr)
        {
            cout << "-> " << edge->getTargetId() << " ";
            edge = edge->getNextEdge();
        }
        cout << "\t Grau: " << node->getDegree() << endl;
        node = node->getNextNode();
    }

    return;
}

void Graph::printGraph2()
{
    cout << "\nImprimindo grafo\n";

    cout << "Ordem: " << this->getOrder() << endl;

    Node *node = first_node;
    while (node != nullptr)
    {
        cout << "No - " << node->getId();
        Edge *edge = node->getFirstEdge();
        while (edge != nullptr)
        {
            cout << " -> " << edge->getTargetId();
            edge = edge->getNextEdge();
        }
        cout << endl;
        node = node->getNextNode();
    }
}

void Graph::printGraph_Dot_Not_Directed()
{
    fstream arqDot;
    arqDot.open("output.dot", ios::out | ios::trunc);
    if (!arqDot.is_open())
    {
        arqDot << "\nErro ao abrir o arquivo dot\n";
        exit(1);
    }
    else
    {
        int k = this->getOrder();
        int cont = 0;
        Node *node = first_node;
        int *vet = new int[k];

        for (int i = 0; i < k; i++)
            vet[i] = 0;

        arqDot << "graph{\n";
        while (node != nullptr)
        {
            vet[cont] = node->getId();
            Edge *edge = node->getFirstEdge();
            while (edge != nullptr)
            {
                int v = edge->getTargetId();
                if (!findEdge(vet, cont, v))
                    arqDot << node->getId() << "--" << edge->getTargetId() << "\n";

                edge = edge->getNextEdge();
            }
            node = node->getNextNode();
            cont++;
        }
        arqDot << "}\n";
        delete[] vet;
    }

    arqDot.close();
}

void Graph::printGraph_Dot_Directed()
{
    fstream arqDot;
    arqDot.open("output.dot", ios::out | ios::trunc);
    if (!arqDot.is_open())
    {
        arqDot << "\nErro ao abrir o arquivo dot\n";
        exit(1);
    }
    else
    {
        int k = this->getOrder();
        int cont = 0;
        Node *node = first_node;
        int *vet = new int[k];

        for (int i = 0; i < k; i++)
            vet[i] = 0;

        arqDot << "digraph graph{\n";
        while (node != nullptr)
        {
            vet[cont] = node->getId();
            Edge *edge = node->getFirstEdge();
            while (edge != nullptr)
            {
                int v = edge->getTargetId();
                if (!findEdge(vet, cont, v))
                    arqDot << node->getId() << "->" << edge->getTargetId() << ";\n";

                edge = edge->getNextEdge();
            }
            node = node->getNextNode();
            cont++;
        }
        arqDot << "}\n";
        delete[] vet;
    }

    arqDot.close();
}

bool Graph::findEdge(int vet[], int cont, int v)
{
    for (int i = 0; i < cont; i++)
    {
        if (vet[i] == v)
        {
            return true;
        }
    }
    return false;
}

//Busca em profundidade de um Nó dado
void Graph::deepSearch(int id)
{
    vector<int> retorno;
   
    int k = this->getOrder();
    int cont = 0;
    Node *node = this->getNode(id);
    int *vet = new int[k];

    for (int i = 0; i < k; i++)
        vet[i] = 0;

    vet[0] = id;
    cout << "\n -- Árvore em Profundidade -- \n\n";
    auxDeepSearch(node, vet, cont, &retorno);
    this->printGraph_Dot_Not_Directed();
    delete[] vet;

    cout << "\n\n -- Arestas de Retorno -- \n\n";
    for(int i = 0; i < retorno.size(); i++)
    cout<< retorno[i] << " < ";
    cout << "\n\n --- \n";
    
}
//Auxiliar da busca em profundidade de um Nó dado
void Graph::auxDeepSearch(Node *node, int vet[], int cont, vector<int> *retorno)
{
    cout << node->getId() << " > ";
    Edge *edge = node->getFirstEdge();
    while (edge != nullptr)
    {
        if (!findEdge(vet, this->getOrder(), edge->getTargetId()))
        {
            node = this->getNode(edge->getTargetId());
            cont++;                     
            vet[cont] = node->getId(); 
            auxDeepSearch(node, vet, cont, retorno);
            edge = edge->getNextEdge();
            retorno->push_back(node->getId());
        }
        else
            edge = edge->getNextEdge();
    }
}


void Graph::directTransitiveClosing(int id)
{
    int k = this->getOrder();
    vector <int> visited; // Cria um vetor de visitados

    auxDirectTransitiveClosing(id, &visited);

    // Imprime o índice de todos os elementos com valor true do vetor de visitados
    for(int i=0; i<visited.size(); i++){
        cout << visited[i] << " | ";
    }
}

void Graph::auxDirectTransitiveClosing(int id, vector<int> *visited){ 
    Node *node = getNode(id);          
    Edge *edge = node->getFirstEdge(); 

    //Realiza a busca em profundidade para visitar todos os nós acessíveis a partir do nó estabelecido
    while(edge->getNextEdge()!= nullptr){
        visited->push_back(edge->getTargetId());
        auxDirectTransitiveClosing(edge->getTargetId(), visited);
        edge = edge->getNextEdge();
    }
}

void Graph::inDirectTransitiveClosing(int id)
{

}    
void Graph::breadthFirstSearch(ofstream &output_file)
{
}

float Graph::floydMarshall(int idSource, int idTarget)
{

    return 0;
}

float Graph::dijkstra(int idSource, int idTarget)
{

    Node *noSource = getNode(idSource);
    Node *noTarget = getNode(idTarget);

    if(noSource != nullptr && noTarget != nullptr){

        int V = getOrder();
        int INF = 999999999;

        int distance[V];   //  Vetor para os distâncias
        bool visited[V];   //  Vetor para os já visitados

        //Fila de prioridade para os pares distancia e vertice
        priority_queue<pair<int, int>, vector<pair<int,int>>, greater<pair<int,int>>> fp; 
        
        //Inicializador dos vetores visitados e distância
        for(int i = 0; i < V;i++){distance[i] = INF;visited[i]=false;}
        
        distance[idSource] = 0;     //Distância do vertice para ele mesmo

        fp.push(make_pair(distance[idSource], idSource));

        pair<int, int> p = fp.top();

        
        int ver = 0,c_edge = 0;

        while (!fp.empty()){
            
            //Pega o do topo
            pair<int , int> p = fp.top();
            //Obtem o vértice
            int u = p.second;
            //Remove da lista de prioridade
            fp.pop();

            if(visited[u] == false){

                visited[u] = true;
                
                Node *node = getNode(u);
                Edge *edge = node->getFirstEdge();
                
                //Passa por todas as arestas do vertice u
                while (edge != nullptr)
                {   
		    //Para caso não haja pesso a distância será 1 por aresta percorrida
                    if(edge->getWeight() == 0)c_edge = 1;
                    else c_edge = edge->getWeight();
                    
                    ver = edge->getTargetId();
                    

                    if(distance[ver] > (distance[u]+ c_edge)){

                        distance[ver] = (distance[u]+ c_edge);
                        fp.push(make_pair(distance[ver], ver));

                    }
                    edge = edge->getNextEdge();
                }
                
            }
            

        }

    
        return distance[idTarget];

    }else{
        

        if(noSource == nullptr)cout<<"Node source inesistente"<<endl;
        if(noTarget == nullptr)cout <<"Node target inesistente" << endl;
        return 0;
    }
    
    
}

//function that prints a topological sorting
void topologicalSorting()
{
}

void breadthFirstSearch(ofstream &output_file)
{
}
Graph *getVertexInduced(int *listIdNodes)
{

    return nullptr;
}

Graph *agmKuskal()
{
    return nullptr;
}
Graph *agmPrim()
{
    return nullptr;
}
