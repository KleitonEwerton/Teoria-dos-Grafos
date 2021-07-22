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

#define INFINITO 999999;

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

void Graph::cleanVisited()
{
    Node *node = this->getFirstNode();

    while (node != nullptr)
    {
        node->setVisited(false);
        node = node->getNextNode();
    }
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

        arqDot << "digraph{\n";
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
    int n;
    cout << "\nCaminhamento Profundidade destacando as Arestas de retorno\n\n";
    do
    {
        cout << "Informe o numero no nó: ";
        cin >> n;
    } while (!this->searchNode(n));

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
    for (int i = 0; i < retorno.size(); i++)
        cout << retorno[i] << " < ";
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

    Node *node = this->getNode(id);

    cleanVisited(); // Limpa todos os nós visitados.

    auxDirectTransitiveClosing(node);

    // Imprime o id de todos os nós visitados.
    for (node = this->first_node; node != nullptr; node = node->getNextNode())
    {
        if (node->getVisited())
        {
            cout << node->getId() << " | ";
        }
    }
}

// Auxiliar do Fecho Transitivo Direto.
void Graph::auxDirectTransitiveClosing(Node *node)
{
    Edge *edge = node->getFirstEdge();

    while (edge != nullptr)
    {
        if (!this->getNode(edge->getTargetId())->getVisited())
        {
            node->setVisited(true);
            node = this->getNode(edge->getTargetId());
            auxDirectTransitiveClosing(node);
            edge = edge->getNextEdge();
        }
    }
}

void Graph::indirectTransitiveClosing(int id)
{

    Node *target = this->getNode(id); // Nó alvo
    Node *source;                     // Nó através do qual será feita a verificação se target é acessível.

    for (source = this->first_node; source != nullptr; source = source->getNextNode())
    {
        cleanVisited(); // Limpa todos os visitados.

        auxIndirectTransitiveClosing(source);

        // Se terget foi visitado, imprime o id de source.
        if (target->getVisited())
        {
            cout << source->getId() << " | ";
        }
    }
}

// Auxiliar do Fecho Transitivo Indireto.
void Graph::auxIndirectTransitiveClosing(Node *node)
{
    Edge *edge = node->getFirstEdge();

    while (edge != nullptr)
    {
        if (!this->getNode(edge->getTargetId())->getVisited())
        {
            node->setVisited(true);
            node = this->getNode(edge->getTargetId());
            auxDirectTransitiveClosing(node);
            edge = edge->getNextEdge();
        }
    }
}

void Graph::breadthFirstSearch(ofstream &output_file)
{
}

int **Graph::iniciaAnterioresFloyd(int **anteriores, int tam){

    anteriores = new int*[tam];
    for(int i = 0; i < tam;i++)anteriores[i] = new int[tam];
    for (int i = 0; i < tam; i++)for (int j = 0; j < tam; j++)anteriores[i][j] = i;

    return anteriores;

}
int **Graph::iniciaDistanciaFloyd(int **distancia, int tam){
    int INF = INFINITO;
    distancia = new int*[tam];

    for(int i = 0; i < tam;i++)distancia[i] = new int[tam];
    for (int i = 0; i < tam; i++)for (int j = 0; j < tam; j++)distancia[i][j] = INF;
    for(int i = 0; i < tam;i++)distancia[i][i] = 0;

    Node *node;                              
    Edge *edge = nullptr;
    list<int>anterior[tam];

    //Colocando o peso da arestas na matriz, caso o grafo não seja ponderado o valor 1 será atribuido
    for (int i = 1; i < tam; i++){
        node = getNode(i);

        if(node != nullptr)edge = node->getFirstEdge();

        while (edge != nullptr){

            if(!getWeightedEdge())distancia[i][edge->getTargetId()] = 1; 
            else distancia[i][edge->getTargetId()] = edge->getWeight();
               
            edge = edge->getNextEdge();
        }
    }

    return distancia;
}
void Graph::imprimeFloyd(list<int>&antecessor){

    int primeiro;
    primeiro = antecessor.front();
    while (!antecessor.empty())
    {
        cout << primeiro << " --> ";
        antecessor.pop_front();
        primeiro = antecessor.front();
    }
    
}

float Graph::floydMarshall(int idSource, int idTarget){
    Node *noSource;
    Node *noTarget;

    if(idSource == idTarget){
        cout << "\n\nA distância é: " << 0 << endl;
        return 0;
    }

    noSource = getNode(idSource);
    noTarget = getNode(idTarget);

    if(noSource != nullptr && noTarget != nullptr){

        int V = getOrder() + 1;
        int INF = INFINITO;
        int i, j, k;  

        int **dist;
        int **pred;

        dist = iniciaDistanciaFloyd(dist, V);
        pred = iniciaAnterioresFloyd(pred, V);
        
	    //Calculando a distância de todos as posições N^3
        for (k = 0; k < V; k++){
            for (i = 0; i < V; i++)                               
                for (j = 0; j < V; j++)
                    if (dist[i][j] > (dist[i][k] + dist[k][j])){
                        dist[i][j] = dist[i][k] + dist[k][j];
                        pred[i][j] = pred[k][j];}}
                 
        
    int distancia = dist[idSource][idTarget];
    
    //Desalocando a matriz distancia usada
    for(i = 0;i<V;i++){delete [] dist[i];}delete [] dist;
    
    list<int>ordemAcesso;           //Lista para conteer a ordem de acesso dos vertices
                                                   
    ordemAcesso.push_front(idTarget);

    int antecessor =  pred[idSource][idTarget];
    while(antecessor != idSource){
        ordemAcesso.push_front(antecessor);
        antecessor = pred[idSource][antecessor];
    }
    ordemAcesso.push_front(idSource);

    for(i = 0;i<V;i++){delete [] pred[i];}
    delete [] pred;
    
    if(distancia >= INF){cout << "A distância é o infinito "; return INF;}
    if(distancia>0)imprimeFloyd(ordemAcesso);

    cout << "\n\nDistância é: "<< distancia<<endl;
    return distancia;

    }else{

        if(noSource == nullptr)cout<<"Source node does not exist"<<endl;
        if(noTarget == nullptr)cout <<"Target node does not exist" << endl;
        return -1;
    }
}

void Graph::imprimeDijkstra(list<int>antecessor[], int idSource, int idTarget){

    int noAnterior;                                                             //Usado para armazenar o vertice anterior
    list<int>ordemAcesso;                                                       //Lista contendo a ordem de acesso dos vertices

    //Armazena na lista na ordem de acesso dos vertices, apartir de seus anteriores
    ordemAcesso.push_front(idTarget);
    noAnterior = antecessor[idTarget].front();

    while (noAnterior != idSource){
        ordemAcesso.push_front(noAnterior);
        noAnterior = antecessor[noAnterior].front();
    }
    ordemAcesso.push_front(idSource);

    int primeiro;
    primeiro = ordemAcesso.front();
    while (!ordemAcesso.empty())
    {
        cout << primeiro << " --> ";
        ordemAcesso.pop_front();
        primeiro = ordemAcesso.front();
    }
    
}


float Graph::dijkstra(int idSource, int idTarget)
{

    Node *noSource = getNode(idSource);
    Node *noTarget = getNode(idTarget);
    if(idSource == idTarget){
        cout << "\n\nA distância é: " << 0 << endl;
        return 0;
    }
    if(noSource != nullptr && noTarget != nullptr){

        int V = getOrder() + 1;
        int INF = INFINITO;
        int *distance = new int[V];   //  Vetor para os distâncias
        bool *visited = new bool[V];   //  Vetor para os já visitados

        //Fila de prioridade para os pares distancia e vertice
        priority_queue<pair<int, int>, vector<pair<int,int>>, greater<pair<int,int>>> fp; 

        list<int>anterior[V];
        
        //Inicializador dos vetores visitados e distância
        for(int i = 0; i < V;i++){distance[i] = INF;visited[i]=false;}
        
        distance[idSource] = 0;                                                 //Distância do vertice para ele mesmo

        fp.push(make_pair(distance[idSource], idSource));                           

        pair<int, int> p = fp.top();

        int ver = 0,c_edge = 0;
        while (!fp.empty()){
            
            pair<int , int> p = fp.top();                                       //Pega o do topo
            int u = p.second;                                                   //Obtem o vértice

            fp.pop();                                                           //Remove da lista de prioridade
            if(visited[u] == false){                                            
                visited[u] = true;                                              //Marca o vertice como visitado
                
                Node *node = getNode(u);                                        
                Edge *edge = node->getFirstEdge();                              

                while (edge != nullptr){                                        //Passa por todas as arestas do vertice u

		            //Para caso não haja pesso a distância será 1 por aresta percorrida
                    if(edge->getWeight() == 0)c_edge = 1;
                    else c_edge = edge->getWeight();

                    ver = edge->getTargetId();                          

                    if(distance[ver] > (distance[u]+ c_edge)){

                        if(!anterior->empty())anterior->clear();                //Caso já tenha um anterior ele será apagado
                        anterior[ver].push_front(u);                            //Armazena o anterior menos custoso de cada vertice
                        distance[ver] = (distance[u]+ c_edge);                  //Atualiza a distância
                        fp.push(make_pair(distance[ver], ver));                 //Adiciona o vertice na fila de prioridade

                    }edge = edge->getNextEdge();}}} 

        int distancia = distance[idTarget];
        
        delete [] distance;                                                      //Desalocando o vetore usado
        delete [] visited;                                                       //Desalocando o vetore usado
        //Caso a distâcia seja infinita o algoritmo se encerra
        if(distancia >= INF){cout << "A distância é o infinito "; return INF;}
            
        //Imprime todo a lista na ordem de acesso
        if(distancia>0)imprimeDijkstra(anterior, idSource, idTarget);
        
        cout << "\n\nA distância é: " << distancia << endl;
        return distancia;


    }else{
        
        if(noSource == nullptr)cout<<"Source node does not exist"<<endl;
        if(noTarget == nullptr)cout <<"Target node does not exist" << endl;
        return -1;
    }
    
    
}

bool Graph::thisIsCyclic()
{
    if (this->directed)
        return this->isCyclicDirected();
    else
        return this->isCyclic();
}

bool Graph::auxIsCyclic(int nodeId, bool isVisited[], int parentId)
{
    isVisited[nodeId] = true;
    Node *node = getNode(nodeId);
    Edge *edge = node->getFirstEdge();

    while (edge != nullptr)
    {
        if (!isVisited[edge->getTargetId()])
        {
            if (auxIsCyclic(edge->getTargetId(), isVisited, nodeId))
            {
                cout << edge->getTargetId() << " - ";
                return true;
            }
        }
        else if (edge->getTargetId() != parentId)
        {
            return true;
        }

        edge = edge->getNextEdge();
    }

    return false;
}

bool Graph::isCyclic()
{
    int order = this->getOrder();

    bool *isVisited = new bool[order];

    for (int i = 0; i < order; i++)
    {
        isVisited[i] = false;
    }

    Node *node = first_node;
    while (node != nullptr)
    {
        if (!isVisited[node->getId()])
        {
            if (auxIsCyclic(node->getId(), isVisited, -1))
            {
                return true;
            }
        }

        node = node->getNextNode();
    }

    return false;
}

bool Graph::auxIsCyclicDirected(int nodeId, bool isVisited[], bool *isContainedRecusirve)
{
    Node *node = this->getNode(nodeId);
    Edge *edge = node->getFirstEdge();

    if (!isVisited[nodeId])
    {
        isVisited[nodeId] = true;
        isContainedRecusirve[nodeId] = true;

        while (edge != nullptr)
        {
            if (!isVisited[edge->getTargetId()] && auxIsCyclicDirected(edge->getTargetId(), isVisited, isContainedRecusirve))
            {
                return true;
            }
            else if (isContainedRecusirve[edge->getTargetId()])
            {
                return true;
            }

            edge = edge->getNextEdge();
        }
    }

    isContainedRecusirve[nodeId] = false;
    return false;
}

bool Graph::isCyclicDirected()
{
    bool *isVisited = new bool[this->getOrder()];
    bool *isContainedRecusirve = new bool[this->getOrder()];
    Node *node = first_node;

    for (int i = 0; i < this->getOrder(); i++)
    {
        isVisited[i] = false;
        isContainedRecusirve[i] = false;
    }

    while (node != nullptr)
    {
        if (auxIsCyclicDirected(node->getId(), isVisited, isContainedRecusirve))
        {
            return true;
        }

        node = node->getNextNode();
    }

    return false;
}

/**
 * @brief Função recursiva usada por topologicalSort
 * 
 * @param node      ponteiro para o No
 * @param edge      ponterio para a Aresta
 * @param o         ordem do graph
 * @param visited   vetor de Nos visitados
 * @param Stack     Pilha de Nós
 */
void Graph::topologicalSortUtil(Node *node, Edge *edge, int o, bool visited[], stack<int> &Stack)
{
    if (node != nullptr)
    {
        // Marca o nó atual como visitado.
        visited[o] = true;
        node = this->getNode(o);
        edge = node->getFirstEdge();

        // recursivo para todos o vértices adjacentes a ele
        while (edge != nullptr)
        {
            if (!visited[edge->getTargetId()])
                topologicalSortUtil(node, edge, edge->getTargetId(), visited, Stack);

            edge = edge->getNextEdge();
        }

        //Empilha o vértice atual
        if (node != nullptr)
            Stack.push(node->getId());
        // cout << Stack.top() << " : ";
    }
}

/**
 * @brief function that prints a topological sorting 
 * A função para fazer a classificação topológica.
 * Ele usa topologicalSortUtil () recursivamente.
 */
void Graph::topologicalSorting()
{
    cout << "\nOrdenação topologica em Grafo Acíclico Direcionado - DAG\n"
         << endl;

    if (!this->getDirected())
    {
        cout << "Ordenacao Topologica impossivel: Grafo nao-direcionado." << endl;
        return;
    }
    else if (this->thisIsCyclic())
    {
        cout << "Ordenacao Topologica impossivel: Grafo não acíclico." << endl;
        return;
    }
    else
    {
        stack<int> Stack;
        int ordem = this->getOrder();
        Node *node = this->first_node;
        Edge *edge = node->getFirstEdge();

        // Marca todos os vértices como não visitados
        bool *visited = new bool[ordem];
        for (int i = 0; i < ordem; i++)
            visited[i] = false;

        // função auxiliar recursiva para armazenar topológico
        // Classificar começando por todos vértices um por um
        for (int i = 1; i <= ordem; i++)
            if (visited[i] == false) // n vértices
                topologicalSortUtil(node, edge, i, visited, Stack);

        // Imprime o conteudo da pilha

        cout << "Ordenação: ";
        while (!Stack.empty())
        {
            cout << Stack.top() << "  ";
            Stack.pop();
        }
        cout << endl
             << endl;
        delete[] visited;
    }
}

void breadthFirstSearch(ofstream &output_file)
{
}
Graph *getVertexInduced(int *listIdNodes)
{

    return nullptr;
}

Graph *agmKruskal()
{
    return nullptr;
}
Graph *agmPrim()
{
    return nullptr;
}
