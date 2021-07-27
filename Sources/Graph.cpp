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
#include <string>
#include <sstream>

#include "Graph.h"
#include "Node.h"
#include "Edge.h"

int INF = 99999999;

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
    this->position = 0;
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
        novo_node->setPosition(this->position);
        last_node->setNextNode(novo_node);
        last_node = novo_node;
    }
    else
    {
        //caso nao tenha, cria um novo node e ele se torna o ultimo e o primeiro
        Node *novo_node = new Node(id);
        novo_node->setPosition(this->position);
        first_node = novo_node;
        last_node = novo_node;
    }
    this->position = this->position + 1;
}

void Graph::insertEdge(int id, int target_id, float weight)
{
    //cria um ponteiro para o node desejado e o um auxiliar para o node alvo da aresta
    Node *node = getNode(id);
    Node *aux = getNode(target_id);
    
    if(node == nullptr){
        this->insertNode(id);
        node = last_node;
    }
    if(aux == nullptr){
        this->insertNode(target_id);
        aux = last_node;
    }
    //confere se os nodes existem
    if (node != nullptr && aux != nullptr)
    {

        //confere se a aresta já existe
        if (!node->searchEdge(target_id))
        {
            //caso o node exista mas a aresta nao, insere a aresta
            node->insertEdge(target_id, aux->getPosition(), weight);
            
            this->number_edges++;

            // se o grafo for nao-direcionado e nao houver aresta de volta
            if (this->directed == 0 && !aux->searchEdge(id))
            {
                //insere a aresta de volta
                aux->insertEdge(id, node->getPosition(),weight);
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
            return node;
        node = node->getNextNode();
    }

    //retorna o node ou null caso nao encontre
    return nullptr;
}

Node *Graph::getNodePosition(int position)
{
    //cria ponteiro para percorrer a lista de nodes
    Node *node = first_node;

    //encontra o node com o id desejado
    while (node != nullptr)
    {
        if (node->getPosition() == position)
            return node;

        node = node->getNextNode();
    }

    //retorna o node ou null caso nao encontre
    return nullptr;
}

void Graph::cleanVisited()
{
    /**
     * @brief Função para definir todos os nós do grafo como não visitados.
     * 
     */

    Node *node = this->getFirstNode();  // Ponteiro que armazena o endereço de memória do primeiro nó do grafo.

    // Realiza a operação para todos os nós do grafo.
    while (node != nullptr)
    {
        node->setVisited(false);        // Define o nó como não visitado.
        node = node->getNextNode();     // Ponteiro passa a apontar para o próximo nó do grafo.
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
void Graph::deepSearch()
{
    int id;
    cout << "\nCaminhamento Profundidade destacando as Arestas de retorno\n\n";
    do
    {
        cout << "Informe o numero no nó: ";
        cin >> id;
    } while (!this->searchNode(id));

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

void Graph::deepPath(Node *node){
    /**
     * @brief         Função genérica para realizar o caminho em profundidade a partir de um nó.
     * @param node    Nó através do qual será feito o caminho em profundidade.
     * 
     */

    node->setVisited(true);  // Define o nó node como visitado.

    // Operação recursiva para percorrer todos os nós acessíveis a partir de node.
    // Percorre todas as arestas do nó.
    for(Edge *edge = node->getFirstEdge(); edge!=nullptr; edge=edge->getNextEdge()){

        // Verifica se o nó para o qual a aresta aponta foi visitado.
        // Caso não tenha sido, deepPath é chamado novamente passando como parâmetro o nó alvo da aresta.
        if(!getNode(edge->getTargetId())->getVisited()){
            deepPath(getNode(edge->getTargetId()));
        }
    }
}

void Graph::directTransitiveClosing(int id)
{
    /**
     * @brief       Função para identificar e exibir o Fecho Transitivo Direto do grafo direcionado.
     * @param id    Valor que representa o id do nó para o qual será calculado o fecho transitivo direto.
     * 
     */

    Node *node = this->getNode(id);  // Nó para o qual será calculado o fecho transitivo direto.

    this->cleanVisited();            // Chama a função para setar todos os nós do grafo como não visitados.

    // Verifica se o nó node existe.
    if(node!=nullptr){

        deepPath(node);              // Realiza o caminho em profundidade no grafo a partir do nó node.

        // Imprime o id de todos os nós visitados.
        for (node = this->first_node; node != nullptr; node = node->getNextNode())
        {
            if (node->getVisited())
            {
                cout << node->getId() << " | ";
            }
        }
    }
    // Se node não existe, exibe uma mensagem de erro.
    else{
        cout << "Erro! Não existe um nó com o id proposto.";
    }
}

void Graph::indirectTransitiveClosing(int id)
{
    /**
     * @brief       Função para identificar e exibir o Fecho Transitivo Indireto do grafo direcionado.
     * @param id    Valor que representa o id do nó para o qual será calculado o fecho transitivo indireto.
     * 
     */

    Node *target = this->getNode(id);     // Nó alvo que recebe o id passado como parâmetro.
    Node *source = this->getFirstNode();  // Nó através do qual será feita a verificação se target é acessível.

    // Verifica se o nó target existe.
    if(target!=nullptr){
        // Realiza a busca em profundidade para todos os nós do grafo.
        while(source != nullptr){

            this->cleanVisited();         // Chama a função para setar todos os nós do grafo como não visitados.

            deepPath(source);             // Realiza o caminho em profundidade no grafo a partir do nó source.

            // Se target foi visitado no caminho em profundidade, imprime o id de source.
            if (target->getVisited()){
                cout << source->getId() << " | ";
            }
            source = source->getNextNode();
        }
    }
    // Se target não existe, imprime uma mensagem de erro.
    else{
        cout << "Erro! Não existe um nó com o id proposto.";
    }
}

void Graph::breadthFirstSearch(ofstream &output_file)
{
}

int **Graph::iniciaAnterioresFloyd(int **anteriores, int tam){
    /**
     * @brief               Função auxiliar de Floy, para iniciar a matriz dos anteriores
     * @param anteriores    Matriz dos anteriores que será inicializada
     * @param tam           Tamanho da matriz quadratica, recebe um valor i para cada posição
     * 
     */


    anteriores = new int*[tam];                                                     //Aloca a matriz dos anteriores
    for(int i = 0; i < tam;i++)anteriores[i] = new int[tam];                        //Aloca cada posição da matriz anteriores
    for (int i = 0; i < tam; i++)for (int j = 0; j < tam; j++)anteriores[i][j] = i; //Coloca um valor em cada posição da matriz anteriores

    return anteriores;                                                              //Retorna a matriz anteriores

}
int **Graph::iniciaDistanciaFloyd(int **distancia, int tam){
    /**
     * @brief               Função auxiliar de Floyd, para iniciar a posição da matriz de distância contendo os pesos
     * @param   distancia   Vetor de distância a ser inicializado
     * @param   tam         Tamanho da matriz quadratica, recebe a ordem do grafo
     * 
     */
    distancia = new int*[tam];                                                      //Alocando a matriz distância
    for(int i = 0; i < tam;i++)distancia[i] = new int[tam];                         //Aloca cada posição da matriz distância
    for (int i = 0; i < tam; i++)for (int j = 0; j < tam; j++)distancia[i][j] = INF;//Atribuindo a distância de infinito para todas a posições
    for(int i = 0; i < tam;i++)distancia[i][i] = 0;                                 //Atribuindo a distância de 0 do vertice para ele mesmo

    Node *node = nullptr;                             
    Edge *edge = nullptr;
    list<int>anterior[tam];                                                         //Lista de anteriores

    for (int i = 0; i < tam; i++){                                                  //Colocando o peso das arestas na matriz, caso o grafo não seja ponderado o valor 1 será atribuido para cada salto
        node = getNodePosition(i);
        edge = node->getFirstEdge();

        while (edge != nullptr){

            if(!getWeightedEdge())distancia[i][edge->getTargetPosition()] = 1;      //Adiciona 1 para cada salto caso o grafo não seja ponderado
            else distancia[i][edge->getTargetPosition()] = edge->getWeight();       //Adiciona o peso nessa posição da matriz
            edge = edge->getNextEdge();                                             //Avança a aresta
        }
    }
    return distancia;                                                               //Retorna a matriz distância
}
void Graph::imprimeFloyd(list<int>&antecessor){

    int primeiro = antecessor.front();
    Node *no = nullptr;

    while (!antecessor.empty())
    {   no = getNodePosition(primeiro);
        cout << no->getId() << " --> ";
        antecessor.pop_front();
        primeiro = antecessor.front();
    }
    
}

float Graph::floydMarshall(int idSource, int idTarget){
    /**
     * @brief               Função de busca de caminho minimo  usando o algoritmo de Floyd-Marshall
     * @param   idSource    ID do nó de origem
     * @param   idTarget    ID do nó de destino
     * 
     */
    Node *noSource = nullptr, *noTarget = nullptr;

    if(idSource == idTarget){cout << "\n\nA distância é: " << 0 << endl;return 0;}

    noSource = getNode(idSource);                                                   //Busca o no Sorce
    noTarget = getNode(idTarget);                                                   //Busca o no Target

    if(noSource != nullptr && noTarget != nullptr){

        int V = getOrder(),i, j, k, distancia,antecessor;
        int **dist, **pred;

        dist = iniciaDistanciaFloyd(dist, V);                                       //Inicia a matriz de distância
        pred = iniciaAnterioresFloyd(pred, V);                                      //Inicia a matriz de anteriores

        for (k = 0; k < V; k++)                                                     //Calculando a distância de todas as posições
            for (i = 0; i < V; i++)                               
                for (j = 0; j < V; j++)
                    if (dist[i][j] > (dist[i][k] + dist[k][j])){
                        dist[i][j] = dist[i][k] + dist[k][j];
                        pred[i][j] = pred[k][j];                                    //Atualiza a posição do no antercessor dos nos na posição i e j
                    }
        distancia = dist[noSource->getPosition()][noTarget->getPosition()];         //Armazena a distância minima entre os dois nós
        
        for(i = 0;i<V;i++){delete [] dist[i];}delete [] dist;                       //Desalocando a matriz distancia usada
        
        list<int>ordemAcesso;                                                       //Lista para conteer a ordem de acesso dos vertices, de trás para frente
                                                    
        ordemAcesso.push_front(noTarget->getPosition());                            //Adiciona a posição do no Target na filha

        antecessor =  pred[noSource->getPosition()][noTarget->getPosition()];       //Pega a possição do antercessor ao no Source e Target

        while(antecessor != noSource->getPosition()){                               //Loop que adciona na lista todas as posições dos nos do menor caminho
            ordemAcesso.push_front(antecessor);
            antecessor = pred[noSource->getPosition()][antecessor];
        }
        ordemAcesso.push_front(noSource->getPosition());                            //Adiciona a posição do no Source na filha

        for(i = 0;i<V;i++){delete [] pred[i];}                                      //Desalocando a matriz de antecessores
        delete [] pred;
        
        if(distancia >= INF){cout << "A distância é o infinito "; return INF;}      //Encera caso não possua um caminho minimo

        imprimeFloyd(ordemAcesso);
        cout << "\n\nDistância é: "<< distancia<<endl;
        return distancia;

    }else{

        if(noSource == nullptr)cout<<"Source node não existe nesse grafo"<<endl;
        if(noTarget == nullptr)cout <<"Target node não existe nesse grafo" << endl;
        return -1;
    }
}

void Graph::imprimeDijkstra(int antecessor[], int idSource, int idTarget){

    /**
     * @brief               Função auxiliar de disjkstra para imprimir o caminho
     * @param   antecessor  Vetor contendo o antecessor de cada posição
     * @param   idSource    ID do nó de origem
     * @param idTarget      ID do nó de destino   
     */

    int noAnterior,primeiro;                                                        //Usado para armazenar o vertice anterior
    list<int>ordemAcesso;                                                           //Lista contendo a ordem de acesso dos vertices

    ordemAcesso.push_front(idTarget);                                               //Armazena na lista na ordem de acesso dos vertices, apartir de seus anteriores

    noAnterior = antecessor[idTarget];                                              

    while (noAnterior != idSource){
        ordemAcesso.push_front(noAnterior);
        noAnterior = antecessor[noAnterior];
    }
    ordemAcesso.push_front(idSource);
    
    primeiro = ordemAcesso.front();
    Node *no = getNodePosition(primeiro);
    while (!ordemAcesso.empty())
    {   no = getNodePosition(primeiro);
        cout << no->getId() << " --> ";
        ordemAcesso.pop_front();
        primeiro = ordemAcesso.front();
    }
    
}


float Graph::dijkstra(int idSource, int idTarget){
    /**
     * @brief               Função para buscar o caminho minimo usando o algoritmo de dijkstra
     * @param idSource      ID do nó de origem
     * @param idTarget      ID do nó de destino
     */

    if(idSource == idTarget){cout <<"\n\nA distância é: "<< 0 <<endl;return 0;} //Encerá caso seja o mesmo no
    Node *noSource = getNode(idSource), *noTarget = getNode(idTarget);          //Busca o no
    
    if(noSource != nullptr && noTarget != nullptr){

        int pSource = noSource->getPosition(), pTarget= noTarget->getPosition(), distancia = INF, V = getOrder();
        int ver = 0,c_edge = 0, u;

        int *distance = new int[V];                                             //Vetor para os distâncias entre a posição do noSorce e os demais
        int *antec = new int [V];                                               //Vetor para os antecessores
        bool *visited = new bool[V];                                            //Vetor para as posições já visitadas
        for(int i = 0; i < V;i++){distance[i] = INF;visited[i]=false;}          //Inicializador dos vetores visitados e distância
        distance[pSource] = 0;                                                  //Distância do vertice para ele mesmo

        priority_queue<pair<int, int>, vector<pair<int,int>>, greater<pair<int,int>>> fp; //Fila de prioridade para os pares distancia e vertice

        fp.push(make_pair(distance[pSource], pSource));                         //Adiciona o par vetor distância e  

        pair<int, int> p = fp.top();                                            //Adiciona o p na fila de prioridade

        Node *node = nullptr;
        Edge *edge = nullptr;  

        while (!fp.empty()){
            
            pair<int , int> p = fp.top();                                       //Pega o do topo
            u = p.second;                                                       //Obtem o vértice
            fp.pop();                                                           //Remove da lista de prioridade
            if(visited[u] == false){                                            
                visited[u] = true;                                              //Marca o vertice como visitado
                node = getNodePosition(u);
                if(node != nullptr)                                             //Busca o no pela posição
                    edge = node->getFirstEdge();    
                else edge = nullptr;                                            //Pega a primeira aresta do no

                while (edge != nullptr){                                        //Passa por todas as arestas do vertice u

                    if(!getWeightedEdge())c_edge = 1;		                    //Para caso não haja pesso a distância será 1 por salto
                    else c_edge = edge->getWeight();

                    ver = edge->getTargetPosition();                            //Pega a posição do no Target dessa aresta    

                    if(distance[ver] > (distance[u]+ c_edge)){                  //Verifica se a distância é menor
                        antec[ver] = u;                                         //Atualiza o antecessor
                        distance[ver] = (distance[u]+ c_edge);                  //Atualiza a distância
                        fp.push(make_pair(distance[ver], ver));                 //Adiciona o vertice na fila de prioridade

                    }edge = edge->getNextEdge();                                //Avança para o a proxima aresta do vertice
                }
            }
        } 

        distancia = distance[pTarget];
        
        delete [] distance;                                                      //Desalocando o vetore usado
        delete [] visited;                                                       //Desalocando o vetore usado
        if(distancia >= INF){cout << "A distância é o infinito ";                //Caso a distância seja infinita o algoritmo se encerra
            delete [] antec;return INF;}                                         
        
        imprimeDijkstra(antec, pSource, pTarget);                                //Imprime todo a lista na ordem de acesso
        delete [] antec;
        cout << "\n\nA distância é: " << distancia << endl;
        return distancia;


    }else{
        
        if(noSource == nullptr)cout<<"Source node não existe nesse grafo"<<endl;
        if(noTarget == nullptr)cout <<"Target node não existe nesse grafo" << endl;
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
 * @param Stack     Pilha de Nós
 */

void Graph::topologicalSortUtil(Node *node, Edge *edge, stack<int> &Stack)
{
    if (node != nullptr)
    {
        node->setVisited(true);                             // Marca o nó atual como visitado.
        Node *auxNode;                                      // cria nó axiliar
        edge = node->getFirstEdge();
        
        while (edge != nullptr)                            
        {
            auxNode = this->getNode(edge->getTargetId());   // Aponta o no auxiliar para edge
            if(!auxNode->getVisited())
               topologicalSortUtil(auxNode, edge, Stack);   // recursivo para todos o vértices adjacentes a ele

            edge = edge->getNextEdge();
        }

        if (node != nullptr)                                 //Empilha o vértice atual
            Stack.push(node->getId());
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
        Node *node = this->first_node;
        Edge *edge = node->getFirstEdge();        
        this->cleanVisited();                   // Marca todos os vértices como não visitados

        while (node != nullptr)
        {
            if(!node->getVisited())
                topologicalSortUtil(node, edge, Stack);

                node = node->getNextNode();
        }

        cout << "Ordenação: ";
        while (!Stack.empty())
        {
            cout << Stack.top() << "  ";
            Stack.pop();
        }
        cout <<"\n\n";
    }
}

void breadthFirstSearch(ofstream &output_file)
{
}
Graph *Graph::getVertInduz()
{
    cout << "\nDigite os IDs dos vértices que irão compor esse subgrafo separados por espaço simples:\n(Exemplo: 5 6 1 8)";
    
    // Lendo os vértices do subgrafo
    string aux;
    cout << "Vértices: ";
    cin >> aux;

    // Vector para armazenar os ids dos vértices do subgrafo
    vector<int> idvertices;
    idvertices.clear();
    
    // Separando a string
    stringstream ss(aux);
    while(ss >> aux)
    {
        if(this->searchNode(stoi(aux)))
            idvertices.push_back(stoi(aux));
        else
            cout << "O vértice " << aux << " é inválido, pois não está no Grafo" << endl;
    }
    
    // Criar o subgrafo vértice induzido
    Graph *subgrafo = new Graph(idvertices.size(), this->getDirected(), this->getWeightedEdge(), this->getWeightedNode());
    for(int i = 0; i < idvertices.size(); i++)
    {

    }
    
    return nullptr;
}

// Estrutura e funções auxiliares para os algoritmos de Árvore Geradora Mínima
struct SubArvore
{
    int pai;
    int ordem;
};

// Função para encontrar em qual subárvore está o nó de id n
int qualSubArvore(SubArvore subarvores[], int n)
{
    if (subarvores[n].pai != n)
        subarvores[n].pai = qualSubArvore(subarvores, subarvores[n].pai);
 
    return subarvores[n].pai;
}

// Função para unir duas subárvores de dois nós u e v
void unirSubArvores(SubArvore subarvores[], int u, int v)
{
    // Encontrando os índices das subárvores
    int subU = qualSubArvore(subarvores, u);
    int subV = qualSubArvore(subarvores, v);
 
    // Unindo a menor com a maior
    if (subarvores[subU].ordem < subarvores[subV].ordem)
        subarvores[subU].pai = subV;
    else if (subarvores[subU].ordem > subarvores[subV].ordem)
        subarvores[subV].pai = subU;

    else
    {
        subarvores[subV].pai = subU;
        subarvores[subU].ordem += subarvores[subV].ordem;
    }
}

// ALGORITMO DE KRUSKAL
// para encontrar a Árvore Geradora Mínima
void Graph::agmKruskal()
{
    cout << "\nIniciando a execução do algoritmo de Kruskal..." << endl;

    // 1º PASSO: Vector para armazenar as arestas do grafo

    vector<pair<int, pair<int, int>>> arestas; //vector<peso, noOrigem, noDestino>
    arestas.clear();

    Node *noAux = this->getFirstNode();
    Edge *arestaAux = noAux->getFirstEdge();

    int u = noAux->getId(); // id do nó de origem
    int v = arestaAux->getTargetId(); //id do nó destino

    // Percorrer todas as arestas do Grafo
    for(int i = 1; i < this->getOrder(); i++)
    {
        cout << "id" << i << ": " << this->getNodePosition(i)->getId() << endl;

        while(arestaAux != nullptr)
        {
            // Coloca a aresta no vetor de arestas
            arestas.push_back({arestaAux->getWeight(), {u, v}});

            // Atualiza os auxiliares se a aresta não for null
            arestaAux = arestaAux->getNextEdge();
            if(arestaAux != nullptr)
            {
                v = arestaAux->getTargetId();
            }
        }

        noAux = this->getNodePosition(i);
        arestaAux = noAux->getFirstEdge();
        u = noAux->getId();
        v = arestaAux->getTargetId();
    }

    for(int j = 0; j < arestas.size(); j++)
    {
        cout << "arestas" << j << "peso: " << arestas[j].first << endl;
        cout << "arestas" << j << "par: " << arestas[j].second.first << " -- " << arestas[j].second.second << endl;

    }

    // Conferir se o vetor de arestas contém todas as arestas do grafo
    if(arestas.size() != this->getNumberEdges())
        {
            cout << "\nO vector arestas não foi preenchido corretamente" << endl;
            cout << "arestas.size = " << arestas.size() << " enquanto getNumberEdges = " << this->getNumberEdges() << endl;
            cout << "Encerrando a execução" << endl;
            return; 
        }

    cout << "1º passo concluído com sucesso" << endl;

    // 2º PASSO: Ordenar as arestas por peso do menor para o maior

    sort(arestas.begin(), arestas.end());
    cout << "2º passo concluído com sucesso" << endl;

    // 3º PASSO: Criar subávores cada uma contendo um nó isolado

    int V = this->getOrder();
    SubArvore* subarvores = new SubArvore[(V * sizeof(SubArvore))]; // vetor para armazenar todas as subárvores

    for(int i = 0; i < V; i++) 
    {
        subarvores[i].pai = i;
        subarvores[i].ordem = 1;
    }
    cout << "3º passo concluído com sucesso" << endl;

    // 4º PASSO: Montar a Árvore Geradora Mínima

    vector<int> agm; // vetor com o índice das arestas da árvore geradora mínima
    agm.clear();

    // Iterar até atingir condição de parada
    int cont = 0;
    int i = arestas.size();
    while(agm.size() < V - 1 && i != 0)
    {
        pair<int, int> proxAresta = arestas[cont].second;
        i --;
        int u = proxAresta.first;
        int v = proxAresta.second;

        // Se u e v não estão na mesma subárvore
        if(qualSubArvore(subarvores, u) != qualSubArvore(subarvores, v))
        {
            agm.push_back(cont);
            unirSubArvores(subarvores, u, v);
            cont ++;
        }
    }
    cout << "4º passo concluído com sucesso" << endl;

    // 5º PASSO: Imprimir a Árvore Geradora Mínima

    cout << "\nÁRVORE GERADORA MÍNIMA via Kruskal\n" << endl;
    cout << "graph {" << endl;
    for(int i = 0; i < agm.size(); i++)
    {
        cout << "  " << arestas[agm[i]].second.first << " -- " << arestas[agm[i]].second.second;
        cout << " [label = " << arestas[agm[i]].first << endl;
    }
    cout << "}" << endl;

    return;
}
Graph *Graph::agmPrim()
{
    return nullptr;
}
