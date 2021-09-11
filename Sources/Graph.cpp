#include <algorithm>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <ctime>
#include <float.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <math.h>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <stack>
#include <string>
#include <time.h>
#include <vector>

#include "Edge.h"
#include "Graph.h"
#include "Node.h"

int INF = 99999999;
int D = 3; // restrição de grau

/**
 * @brief Struct utilizada para facilitar a manipulação dos valores no vetor de médias
 * 
 */
struct media
{
    float soma;
    float numSolucoes;
    float media;
};

int escolheAlfa(vector<float> &prob);
void atualizaMedias(vector<media> &medias, int solucao, vector<float> &alfas, float alfa);
void atualizaProbabilidades(vector<media> &medias, vector<float> &prob, vector<float> &solBest, vector<float> &q);
int randAlfa(float alfa, int tam_vetor);
void inicializaVetores(vector<float> &prob, vector<media> &medias, int &numAlfas);
void imprimirMatriz(vector<vector<pair<int, int>>> &matriz);
int pesoTotalAGM(vector<pair<int, int>> &pesoNoTotal);

using namespace std;

//**************************************************************************************************
//! * Defining the Graph's methods
//**************************************************************************************************/

//! Constructor
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

//! Destructor
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

//! Getters --------------------------------------------------------------------------------------------

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

// Função para gerar um Subgrafo Vértice Induzido
Graph *Graph::getVertInduz()
{
    cout << "\nVocê deseja rodar para o Grafo inteiro ou para um subgrafo?" << endl;
    cout << "1 - Grafo inteiro\n2 - Subgrafo\nSua opção: ";
    int opcao;
    cin >> opcao;

    if (opcao == 2)
    {
        cout << "\nDigite os IDs dos vértices que irão compor esse subgrafo separados por ponto-vírgula (Exemplo: 5;6;1;8):" << endl;

        // Lendo os vértices do subgrafo
        string aux;
        cout << "Vértices: ";
        cin >> aux;

        // Vector para armazenar os ids dos vértices do subgrafo
        vector<int> idvertices;
        idvertices.clear();

        // Separando a string
        stringstream ss(aux);
        while (getline(ss, aux, ';'))
        {
            if (this->searchNode(stoi(aux)))
                idvertices.push_back(stoi(aux));
            else
                cout << "O vértice " << aux << " é inválido, pois não está no Grafo" << endl;
        }

        // Criar o subgrafo vértice induzido
        Graph *subgrafo = new Graph(idvertices.size(), this->getDirected(), this->getWeightedEdge(), this->getWeightedNode());

        // Inserindo as arestas correspondentes no subgrafo
        this->cleanVisited();
        for (int i = 0; i < idvertices.size(); i++)
        {
            for (int j = i + 1; j < idvertices.size(); j++)

                // Verificar se a aresta realmente existe no grafo original
                if ((!this->getNode(idvertices[j])->getVisited()) && this->getNode(idvertices[i])->searchEdge(idvertices[j]))
                {
                    Edge *aux = this->getNode(idvertices[i])->getEdge(idvertices[j]);
                    subgrafo->insertEdge(idvertices[i], idvertices[j], aux->getWeight());
                }
                else
                    subgrafo->insertNode(idvertices[j]);

            this->getNode(idvertices[i])->setVisited(true);
        }

        cout << "\nO Subgrafo X foi gerado com sucesso! ";
        cout << "(Ordem = " << subgrafo->getOrder() << " e Num de Arestas = " << subgrafo->getNumberEdges() << ")" << endl;

        return subgrafo;
    }
    else
        return this;
}

//! Other methods --------------------------------------------------------------------------------------------
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

    if (node == nullptr)
    {
        this->insertNode(id);
        node = last_node;
    }
    if (aux == nullptr)
    {
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
                aux->insertEdge(id, node->getPosition(), weight);
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

void Graph::cleanVisited()
{
    /**
     * @brief Função para definir todos os nós do grafo como não visitados.
     * 
     */

    Node *node = this->getFirstNode(); // Ponteiro que armazena o endereço de memória do primeiro nó do grafo.

    // Realiza a operação para todos os nós do grafo.
    while (node != nullptr)
    {
        node->setVisited(false);    // Define o nó como não visitado.
        node = node->getNextNode(); // Ponteiro passa a apontar para o próximo nó do grafo.
    }
}

//! FUNÇÕES AUXILIARES --------------------------------------------------------------------------------------------

// Estrutura e funções auxiliares para o algoritmo de Kruskal
struct SubArvore
{
    int pai;
    int ordem;
};

// Função para encontrar em qual subárvore está o nó de id n. Usada no Kruskal
int qualSubArvore(SubArvore subarvores[], int n)
{
    if (subarvores[n].pai != n)
        subarvores[n].pai = qualSubArvore(subarvores, subarvores[n].pai);

    return subarvores[n].pai;
}

// Função para unir duas subárvores de dois nós u e v. Usada no Kruskal
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

// Função para auxiliar o algoritmo de Prim. Retorna a posição do nó com menor custo de vizinhança que não esteja na agm
int posicaoMenor(vector<int> &custoViz, vector<bool> &naAGM)
{
    int min = INF;
    int pos;
    bool tem_pos = false;
    for (int i = 0; i < custoViz.size(); i++)
    {
        if (custoViz[i] < min && naAGM[i] == false)
        {
            min = custoViz[i];
            pos = i;
            tem_pos = true;
        }
    }
    if (tem_pos)
        return pos;
    else
    {
        for (int i = 0; i < custoViz.size(); i++)
        {
            if (custoViz[i] == min && naAGM[i] == false)
            {
                min = custoViz[i];
                pos = i;
                tem_pos = true;
                return pos;
            }
        }
    }
    return pos;
}

// Função para imprimir a AGM via Prim
void imprimirPrim(Graph *subgrafo, vector<int> &agm, ofstream &outFile)
{
    int peso = 0;
    cout << "\nÁRVORE GERADORA MÍNIMA via Prim\n"
         << endl;
    cout << "graph {" << endl;
    for (int i = 0; i < subgrafo->getOrder(); i++)
    {
        if (agm[i] != INF)
        {
            int id_destino = subgrafo->getNodePosition(i)->getId();
            if (agm[i] == id_destino)
                cout << "  " << agm[i] << endl;
            else
            {
                cout << "  " << agm[i] << " -- " << id_destino;
                cout << " [label = " << subgrafo->getNode(agm[i])->getEdge(id_destino)->getWeight() << "]" << endl;
                peso += subgrafo->getNode(agm[i])->getEdge(id_destino)->getWeight();
            }
        }
    }
    cout << "}" << endl;
    cout << "\nPeso da AGM: " << peso << endl;
    cout << "\nPrim concluído com sucesso!" << endl;

    cout << "\nDeseja imprimir essa AGM no arquivo de saída? (1 - Sim ou 0 - Não)" << endl;
    int op;
    do
    {
        cout << "Sua opção: ";
        cin >> op;
    } while (op != 1 && op != 0);

    if (op == 1)
    {
        outFile << "graph {" << endl;
        for (int i = 0; i < subgrafo->getOrder(); i++)
        {
            if (agm[i] != INF)
            {
                int id_destino = subgrafo->getNodePosition(i)->getId();
                if (agm[i] == id_destino)
                    outFile << "  " << agm[i] << endl;
                else
                {
                    outFile << "  " << agm[i] << " -- " << id_destino;
                    outFile << " [label = " << subgrafo->getNode(agm[i])->getEdge(id_destino)->getWeight() << "]" << endl;
                }
            }
        }
        outFile << "}" << endl;
        cout << "\nImpressão concluída! Cheque o arquivo de saída" << endl;
    }
}

// Função para imprimir a AGM via Kruskal
void imprimirKruskal(vector<pair<int, pair<int, int>>> &arestas, vector<int> &agm, ofstream &outFile)
{
    int peso = 0;
    cout << "\nÁRVORE GERADORA MÍNIMA via Kruskal\n"
         << endl;
    cout << "graph {" << endl;
    for (int i = 0; i < agm.size(); i++)
    {
        if (arestas[agm[i]].second.first == arestas[agm[i]].second.second)
            cout << "  " << arestas[agm[i]].second.first << endl;
        else
        {
            cout << "  " << arestas[agm[i]].second.first << " -- " << arestas[agm[i]].second.second;
            cout << " [label = " << arestas[agm[i]].first << "]" << endl;
            peso += arestas[agm[i]].first;
        }
    }
    cout << "}" << endl;
    cout << "\nPeso da AGM: " << peso << endl;
    cout << "\nKruskal concluído com sucesso!" << endl;

    cout << "\nDeseja imprimir essa AGM no arquivo de saída? (1 - Sim ou 0 - Não)" << endl;
    int op;
    do
    {
        cout << "Sua opção: ";
        cin >> op;
    } while (op != 1 && op != 0);

    if (op == 1)
    {
        outFile << "graph {" << endl;
        for (int i = 0; i < agm.size(); i++)
        {
            if (arestas[agm[i]].second.first == arestas[agm[i]].second.second)
                outFile << "  " << arestas[agm[i]].second.first << endl;
            else
            {
                outFile << "  " << arestas[agm[i]].second.first << " -- " << arestas[agm[i]].second.second;
                outFile << " [label = " << arestas[agm[i]].first << "]" << endl;
            }
        }
        outFile << "}" << endl;
        cout << "\nImpressão concluída! Cheque o arquivo de saída" << endl;
    }
}

// FUNCIONALIDADES --------------------------------------------------------------------------------------------

/**
     * @brief          Função para identificar e exibir o Fecho Transitivo Direto do grafo direcionado.
     * @param id       Valor que representa o id do nó para o qual será calculado o fecho transitivo direto.
     * @param outFile  Arquivo de saída em .dot.
     * 
     */
void Graph::directTransitiveClosing(int id, ofstream &outFile)
{

    Node *node = this->getNode(id); // Nó para o qual será calculado o fecho transitivo direto.

    this->cleanVisited(); // Chama a função para setar todos os nós do grafo como não visitados.

    // Verifica se o nó node existe.
    if (node != nullptr)
    {

        int entrada;

        cout << "Deseja salvar o resultado em um arquivo de saída? " << endl;
        cout << "Digite: \n1 - Sim\n2 - Não" << endl;
        cout << "Entrada: ";
        cin >> entrada;

        if (entrada == 1)
        {
            outFile << "digraph{\n";
        }

        cout << "Fecho transitivo direto: ";

        deepPath(node); // Realiza o caminho em profundidade no grafo a partir do nó node.

        // Imprime o id de todos os nós visitados.
        for (node = this->first_node; node != nullptr; node = node->getNextNode())
        {
            Edge *edge = node->getFirstEdge();

            if (node->getVisited())
            {
                cout << node->getId() << " | ";

                // Imprime no arquivo .dot a árvore do fecho transitivo direto.
                if (entrada == 1)
                {
                    while (edge != nullptr)
                    {

                        int id = edge->getTargetId();
                        outFile << node->getId() << "->" << edge->getTargetId() << ";\n";
                        edge = edge->getNextEdge();
                    }
                }
            }
        }

        if (entrada == 1)
        {
            outFile << "}\n";
        }
    }
    // Se node não existe, exibe uma mensagem de erro.
    else
    {
        cout << "Erro! Não existe um nó com o id proposto.";
    }
}

/**
     * @brief          Função para identificar e exibir o Fecho Transitivo Indireto do grafo direcionado.
     * @param id       Valor que representa o id do nó para o qual será calculado o fecho transitivo indireto.
     * @param outFile  Arquivo de saída em .dot.
     * 
     */
void Graph::indirectTransitiveClosing(int id, ofstream &outFile)
{

    Node *target = this->getNode(id);    // Nó alvo que recebe o id passado como parâmetro.
    Node *source = this->getFirstNode(); // Nó através do qual será feita a verificação se target é acessível.

    // Verifica se o nó target existe.
    if (target != nullptr)
    {

        int entrada;

        cout << "Deseja salvar o resultado em um arquivo de saída? " << endl;
        cout << "Digite: \n1 - Sim\n2 - Não" << endl;
        cout << "Entrada: ";
        cin >> entrada;

        if (entrada == 1)
        {
            outFile << "digraph{\n";
        }

        cout << "Fecho transitivo indireto: ";

        // Realiza a busca em profundidade para todos os nós do grafo.
        while (source != nullptr)
        {

            this->cleanVisited(); // Chama a função para setar todos os nós do grafo como não visitados.

            deepPath(source); // Realiza o caminho em profundidade no grafo a partir do nó source.

            // Se target foi visitado no caminho em profundidade, imprime o id de source.
            if (target->getVisited())
            {

                cout << source->getId() << " | ";

                if (entrada == 1)
                {
                    outFile << source->getId() << ";\n"; // Imprime no arquivo .dot o id do nó source.
                }
            }
            source = source->getNextNode();
        }

        if (entrada == 1)
        {
            outFile << "}\n";
        }
    }
    // Se target não existe, imprime uma mensagem de erro.
    else
    {
        cout << "Erro! Não existe um nó com o id proposto.";
    }
}

/**
 * @brief    Função para buscar o caminho minimo usando o algoritmo de dijkstra
 */
float Graph::dijkstra(ofstream &outFile)
{

    int idSource, idTarget;
    Node *noSource, *noTarget;
    string idS, idT;
    try
    { //Try para verificar se os parâmetros passados são IDs inteiros
        cout << "Digite o node Source" << endl;
        cin >> idS;
        cout << "Digite o node Target" << endl;
        cin >> idT;
        idSource = stoi(idS);
        idTarget = stoi(idT);
    }
    catch (const exception &e)
    {
        cout << "Parâmetros inválidos." << endl;
        return 0;
    }

    if (idSource == idTarget)
    {
        cout << "\n\nA distância é: " << 0 << endl;
        return 0;
    } //Encerá caso seja o mesmo no

    noSource = getNode(idSource); //Busca o no
    noTarget = getNode(idTarget); //Busca o no

    if (noSource != nullptr && noTarget != nullptr)
    {

        int pSource = noSource->getPosition(), pTarget = noTarget->getPosition(), distancia = INF, V = getOrder();
        int ver = 0, c_edge = 0, u;

        int *distance = new int[V];  //Vetor para os distâncias entre a posição do noSorce e os demais
        int *antec = new int[V];     //Vetor para os antecessores
        bool *visited = new bool[V]; //Vetor para as posições já visitadas
        for (int i = 0; i < V; i++)
        {
            distance[i] = INF;
            visited[i] = false;
        }                      //Inicializador dos vetores visitados e distância
        distance[pSource] = 0; //Distância do vertice para ele mesmo

        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> fp; //Fila de prioridade para os pares distancia e vertice

        fp.push(make_pair(distance[pSource], pSource)); //Adiciona o par vetor distância e

        pair<int, int> p = fp.top(); //Adiciona o p na fila de prioridade

        Node *node = nullptr;
        Edge *edge = nullptr;

        while (!fp.empty())
        {

            pair<int, int> p = fp.top(); //Pega o do topo
            u = p.second;                //Obtem o vértice
            fp.pop();                    //Remove da lista de prioridade
            if (visited[u] == false)
            {
                visited[u] = true; //Marca o vertice como visitado
                node = getNodePosition(u);
                if (node != nullptr) //Busca o no pela posição
                    edge = node->getFirstEdge();
                else
                    edge = nullptr; //Pega a primeira aresta do no

                while (edge != nullptr)
                { //Passa por todas as arestas do vertice u

                    if (!getWeightedEdge())
                        c_edge = 1; //Para caso não haja pesso a distância será 1 por salto
                    else
                        c_edge = edge->getWeight();

                    ver = edge->getTargetPosition(); //Pega a posição do no Target dessa aresta

                    if (distance[ver] > (distance[u] + c_edge))
                    {                                           //Verifica se a distância é menor
                        antec[ver] = u;                         //Atualiza o antecessor
                        distance[ver] = (distance[u] + c_edge); //Atualiza a distância
                        fp.push(make_pair(distance[ver], ver)); //Adiciona o vertice na fila de prioridade
                    }
                    edge = edge->getNextEdge(); //Avança para o a proxima aresta do vertice
                }
            }
        }

        distancia = distance[pTarget];

        delete[] distance; //Desalocando o vetore usado
        delete[] visited;  //Desalocando o vetore usado

        if (distancia < INF)
            saidaDijkstra(antec, pSource, pTarget, outFile); //Imprime todo a lista na ordem de acesso

        delete[] antec;
        cout << "\n\nA distância é: " << distancia << endl;
        return distancia;
    }
    else
    {

        if (noSource == nullptr)
            cout << "Source node não existe nesse grafo" << endl;
        if (noTarget == nullptr)
            cout << "Target node não existe nesse grafo" << endl;
        return -1;
    }
}

/**
 * @brief Função de busca de caminho minimo usando o algoritmo de Floyd-Warshall
 * 
 * @param outFile 
 * @return float 
 */
float Graph::floydWarshall(ofstream &outFile)
{
    int idSource, idTarget;
    Node *noSource, *noTarget;
    string idS, idT;
    try
    { //Try para verificar se os parâmetros passados são IDs inteiros
        cout << "Digite o node Source" << endl;
        cin >> idS;
        cout << "Digite o node Target" << endl;
        cin >> idT;
        idSource = stoi(idS);
        idTarget = stoi(idT);
    }
    catch (const exception &e)
    {
        cout << "Parâmetros inválidos" << endl;
        return 0;
    }

    if (idSource == idTarget)
    {
        cout << "\n\nA distância é: " << 0 << endl;
        return 0;
    }

    noSource = getNode(idSource); //Busca o no Sorce
    noTarget = getNode(idTarget); //Busca o no Target

    if (noSource != nullptr && noTarget != nullptr)
    {

        int V = getOrder(), i, j, k, distancia;
        int **dist, **pred;

        dist = iniciaDistanciaFloyd(dist, V);  //Inicia a matriz de distância
        pred = iniciaAnterioresFloyd(pred, V); //Inicia a matriz de anteriores

        for (k = 0; k < V; k++) //Calculando a distância de todas as posições
            for (i = 0; i < V; i++)
                for (j = 0; j < V; j++)
                    if (dist[i][j] > (dist[i][k] + dist[k][j]))
                    {
                        dist[i][j] = dist[i][k] + dist[k][j];
                        pred[i][j] = pred[k][j]; //Atualiza a posição do no antercessor dos nos na posição i e j
                    }
        distancia = dist[noSource->getPosition()][noTarget->getPosition()]; //Armazena a distância minima entre os dois nós

        for (i = 0; i < V; i++)
        {
            delete[] dist[i];
        }
        delete[] dist; //Desalocando a matriz distancia usada

        if (distancia < INF)
            saidaFloyd(pred, noSource, noTarget, outFile);

        for (i = 0; i < V; i++)
        {
            delete[] pred[i];
        } //Desalocando a matriz de antecessores
        delete[] pred;

        cout << "\n\nDistância é: " << distancia << endl;
        return distancia;
    }
    else
    {

        if (noSource == nullptr)
            cout << "Source node não existe nesse grafo" << endl;
        if (noTarget == nullptr)
            cout << "Target node não existe nesse grafo" << endl;
        return -1;
    }
}

/**
 * @brief ALGORITMO DE PRIM para encontrar a Árvore Geradora Mínima
 * 
 * @param subgrafo 
 * @param outFile 
 */
void Graph::agmPrim(Graph *subgrafo, ofstream &outFile)
{
    cout << "\nIniciando a execução do algoritmo de Prim..." << endl;

    // 1º PASSO: Organizar os custos das vizinhanças dos nós em um vector

    // Vector para armazenar os custoViz dos nós do subgrafo. O índice no vector é compatível com a posição do nó no subgrafo
    vector<int> custoViz;
    custoViz.clear();

    // Vector para checar se o nó já foi inserido na agm
    vector<bool> naAGM(subgrafo->getOrder(), false);

    // O primeiro nó do vector será inicializado com custoViz = 0
    custoViz.push_back(0);

    // Os demais nós serão inicializados com custoViz = INFINITO
    for (int i = 1; i < subgrafo->getOrder(); i++)
        custoViz.push_back(INF);

    cout << "1º passo concluído com sucesso" << endl;

    // 2º PASSO: Criar Arvore Geradora Minima -> vetor com os pais de cada nó da agm ou INF caso nao tenha pai

    // Os índices da agm corresponderão à posição do nó no subgrafo
    // A raiz da agm, indice 0, será o primeiro nó do subgrafo, portanto não terá pai
    vector<int> agm(subgrafo->getOrder(), INF);

    cout << "2º passo concluído com sucesso" << endl;

    // 3º PASSO: Iterar pelos vértices verificando o custoViz e inserindo na agm

    int cont = 0;
    while (cont < subgrafo->getOrder())
    {
        // Pega o nó com menor custoViz que ainda não está na agm
        int pos_menor = posicaoMenor(custoViz, naAGM);         // Posição do nó
        int u = subgrafo->getNodePosition(pos_menor)->getId(); // ID do nó
        // Atualiza naAGM, pois, nessa iteração, u será colocado na agm
        naAGM[pos_menor] = true;

        // Iterar pelos nós v adjacentes a u e verificar se o peso da aresta entre eles é menor que o seu custoViz
        Edge *aux = subgrafo->getNode(u)->getFirstEdge();
        if (aux == nullptr) // nó não tem arestas
            agm[pos_menor] = u;
        else
        {
            while (aux != nullptr)
            {
                int v = aux->getTargetId();                      // ID de v
                int pos_v = subgrafo->getNode(v)->getPosition(); // posição de v
                if (!naAGM[pos_v])                               // executa caso o nó v ainda não esteja na agm
                {
                    // Se o peso da aresta (u, v) for menor que o custoViz de v, atualiza o custoViz com o valor do peso
                    if (aux->getWeight() < custoViz[pos_v])
                    {
                        custoViz[pos_v] = aux->getWeight();
                        // Atualiza o pai de v na agm
                        agm[pos_v] = u;
                    }
                }
                aux = aux->getNextEdge();
            }
        }
        cont++;
    }

    cout << "3º passo concluído com sucesso" << endl;

    // 4º PASSO: Imprimir a Árvore Geradora Mínima e seu peso

    imprimirPrim(subgrafo, agm, outFile);

    return;
}

// ALGORITMO DE KRUSKAL
// para encontrar a Árvore Geradora Mínima
void Graph::agmKruskal(Graph *subgrafo, ofstream &outFile)
{
    cout << "\nIniciando a execução do algoritmo de Kruskal..." << endl;

    // 1º PASSO: Vector para armazenar as arestas do grafo

    vector<pair<int, pair<int, int>>> arestas; //vector<peso, noOrigem, noDestino>
    arestas.clear();

    subgrafo->cleanVisited();
    Node *noAux = subgrafo->getFirstNode();
    Edge *arestaAux = noAux->getFirstEdge();

    int u = noAux->getId(); // id do nó de origem
    int v;

    if (arestaAux != nullptr)
        v = arestaAux->getTargetId(); //id do nó destino

    // Percorrer todas as arestas do Grafo
    for (int i = 1; i < subgrafo->getOrder(); i++)
    {
        if (arestaAux == nullptr)
            arestas.push_back({INF, {u, u}});

        while (arestaAux != nullptr)
        {
            // Coloca a aresta no vetor de arestas
            if (!subgrafo->getNode(v)->getVisited())
                arestas.push_back({arestaAux->getWeight(), {u, v}});

            // Atualiza os auxiliares se a aresta não for null
            arestaAux = arestaAux->getNextEdge();
            if (arestaAux != nullptr)
            {
                v = arestaAux->getTargetId();
            }
        }

        noAux->setVisited(true);
        noAux = subgrafo->getNodePosition(i);
        arestaAux = noAux->getFirstEdge();
        u = noAux->getId();
        if (arestaAux != nullptr)
            v = arestaAux->getTargetId();
    }

    cout << "1º passo concluído com sucesso" << endl;

    // 2º PASSO: Ordenar as arestas por peso do menor para o maior

    sort(arestas.begin(), arestas.end());
    cout << "2º passo concluído com sucesso" << endl;

    // 3º PASSO: Criar subávores cada uma contendo um nó isolado

    int V = subgrafo->getOrder();
    SubArvore *subarvores = new SubArvore[(V * sizeof(SubArvore))]; // vetor para armazenar todas as subárvores

    for (int i = 0; i < V; i++)
    {
        subarvores[i].pai = i;
        subarvores[i].ordem = 1;
    }
    cout << "3º passo concluído com sucesso" << endl;

    // 4º PASSO: Montar a Árvore Geradora Mínima

    vector<int> agm; // vetor com o índice associado à posição de cada aresta da árvore geradora mínima no vector 'arestas' do subgrafo
    agm.clear();

    // Iterar até atingir condição de parada
    int cont = 0;
    while (agm.size() < V - 1 && cont < arestas.size())
    {
        pair<int, int> proxAresta = arestas[cont].second;
        int u = proxAresta.first;
        int v = proxAresta.second;

        if (u == v)
            agm.push_back(cont);

        // Se u e v não estão na mesma subárvore
        if (qualSubArvore(subarvores, subgrafo->getNode(u)->getPosition()) != qualSubArvore(subarvores, subgrafo->getNode(v)->getPosition()))
        {
            agm.push_back(cont);
            unirSubArvores(subarvores, subgrafo->getNode(u)->getPosition(), subgrafo->getNode(v)->getPosition());
        }
        cont++;
    }
    cout << "4º passo concluído com sucesso" << endl;

    // 5º PASSO: Imprimir a Árvore Geradora Mínima e seu peso

    imprimirKruskal(arestas, agm, outFile);

    delete[] subarvores;
    return;
}

/**
 * @brief Caminhamento Profundidade destacando as Arestas de retorno
 * 
 * @param outFile Arquivo paraa getação de saida gráfica
 */
void Graph::deepSearch(ofstream &outFile)
{
    // variáveis
    vector<int> retorno;
    vector<int> findG;
    vector<string> graph;
    int id;
    char r;

    cout << "\nCaminhamento Profundidade destacando as Arestas de retorno\n\n";
    do
    {
        cout << "Informe o numero no nó: ";
        cin >> id;
    } while (!this->searchNode(id));

    Node *node = this->getNode(id);
    auxDeepSearch(node, &findG, &retorno, outFile, &graph); //chama a função auxiliar

    cout << "\n -- Árvore em Profundidade -- \n";
    for (int i = 0; i < findG.size(); i++)
        cout << findG[i] << " | ";

    cout << "\n\n -- Arestas de Retorno -- \n";
    for (int i = 0; i < retorno.size(); i++)
        cout << retorno[i] << " | ";
    cout << endl;

    do
    {
        cout << "Gerar Grafo e arquivo de saída(s/n)?\n";
        cin >> r;
    } while (r != 's' && r != 'S' && r != 'n' && r != 'n');

    if (r == 's' || r == 'S')
        printDeepSearch(&graph, outFile);
    retorno.clear();
    findG.clear();
    graph.clear();
}

void Graph::printDeepSearch(vector<string> *graph, ofstream &outFile)
{
    ofstream output;
    output.open("output.dot", ios::out | ios::trunc);
    output << "graph{\n";
    outFile << "graph{\n";
    for (int i = 0; i < graph->size(); i++)
    {
        output << graph->at(i) << endl;
        outFile << graph->at(i) << endl;
    }
    output << "}";
    outFile << "}";
    output.close();
    system("dot -Tpng output.dot -o output.png");
    //outFile.close();
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
        this->cleanVisited(); // Marca todos os vértices como não visitados

        while (node != nullptr)
        {
            if (!node->getVisited())
                topologicalSortUtil(node, edge, Stack);

            node = node->getNextNode();
        }

        cout << "Ordenação: ";
        while (!Stack.empty())
        {
            cout << Stack.top() << "  ";
            Stack.pop();
        }
        cout << "\n\n";
    }
}

//! AUXILIAR METHODS --------------------------------------------------------------------------------------------

/**
 *  @brief         Função genérica para realizar o caminho em profundidade a partir de um nó.
 *  @param node    Nó através do qual será feito o caminho em profundidade.
 *  
 */
void Graph::deepPath(Node *node)
{

    node->setVisited(true); // Define o nó node como visitado.

    // Operação recursiva para percorrer todos os nós acessíveis a partir de node.
    // Percorre todas as arestas do nó.
    for (Edge *edge = node->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge())
    {

        // Verifica se o nó para o qual a aresta aponta foi visitado.
        // Caso não tenha sido, deepPath é chamado novamente passando como parâmetro o nó alvo da aresta.
        if (!getNode(edge->getTargetId())->getVisited())
        {

            deepPath(getNode(edge->getTargetId()));
        }
    }
}

/**
 * @brief Auxiliar da busca em profundidade de um Nó dado
 * 
 * @param node      Ponteiro para o nó
 * @param findG     vetor para os vértices do caminho
 * @param retorno   vetor para as arestas de retorno
 * @param outFile   arquivo texto de saída
 * @param graf      vetor para gravação dos dados impressos no arquivo de saída
 */
void Graph::auxDeepSearch(Node *node, vector<int> *findG, vector<int> *retorno, ofstream &outFile, vector<string> *graf)
{

    findG->push_back(node->getId());
    node->setVisited(true);
    for (Edge *edge = node->getFirstEdge(); edge != nullptr; edge = edge->getNextEdge())
    {
        if (!getNode(edge->getTargetId())->getVisited())
        {
            graf->push_back(to_string(node->getId()) + "--" + to_string(edge->getTargetId()));
            auxDeepSearch(getNode(edge->getTargetId()), findG, retorno, outFile, graf);
        }
    }
    retorno->push_back(node->getId());
}

/**
 * @brief               Função auxiliar para as saidas dos algoritmos de caminho minimo, imprimindo e salvando no arquivo
 * @param   antecessor  Lista contendo os antecessores do caminho minimo, representa a ordem de acesso
 * @param   outFile     Arquivo de saída
 */
void Graph::caminhoMinimo(list<int> &antecessor, ofstream &outFile)
{
    string arco, entrada;                            //Para a escrita no outFile, se for arco '->' se for aresta '--'
    int primeiro = antecessor.front(), tNode, sNode; //Usado para armazenar o primeiro vertice, e auxiliar na escrita no arquivo dot

    cout << "\nDeseja salvar a saída?\n1 para sim.\nQualquer outra opção para não" << endl;
    cout << "Sua opção: ";
    cin >> entrada;
    if (getDirected())
    {
        if (entrada == "1")
            outFile << "\ndigraph{ \n";
        arco = " -> ";
    }
    else
    {
        if (entrada == "1")
            outFile << "\ngraph{ \n";
        arco = " -- ";
    }
    Node *no = getNodePosition(primeiro);
    sNode = no->getId();
    Node *noAux = nullptr; //No auxiliar
    Edge *edge = nullptr;  //Edge auxiliar para pegar o peso
    cout << "\nCAMINHO MINIMO\n"
         << endl;
    while (!antecessor.empty())
    {                                   //Passa por toda a lista de ordem de acesso em buscando o ID
        no = getNodePosition(primeiro); //no recebe o node que é o primeiro no caminho minimo
        tNode = no->getId();

        if (tNode != sNode && entrada == "1") //Para escrever no arquivo dot o caminho, caso possua peso busca esse peso
            if (getWeightedEdge())
            {
                edge = noAux->getEdge(tNode); //Busca a aresta para pegar o seu peso
                outFile << sNode << arco << tNode << "[label=" << edge->getWeight() << "]\n";
            }
            else
                outFile << sNode << arco << tNode << "\n";

        sNode = tNode; //Atualiza o valor do sNode

        cout << no->getId() << arco;   //Imprime para o usuário
        antecessor.pop_front();        //Remove o primeiro elemento da lista dos antecessor
        primeiro = antecessor.front(); //Atualiza o valor do primeiro
        noAux = no;                    //Atualiza o valor do noAux
    }
    if (entrada == "1")
        outFile << "}";
}

/**
 * @brief               Função auxiliar de Floy, para iniciar a matriz dos anteriores
 * @param anteriores    Matriz dos anteriores que será inicializada
 * @param tam           Tamanho da matriz quadratica, recebe um valor i para cada posição
 * 
 */
int **Graph::iniciaAnterioresFloyd(int **anteriores, int tam)
{
    anteriores = new int *[tam]; //Aloca a matriz dos anteriores
    for (int i = 0; i < tam; i++)
        anteriores[i] = new int[tam]; //Aloca cada posição da matriz anteriores
    for (int i = 0; i < tam; i++)
        for (int j = 0; j < tam; j++)
            anteriores[i][j] = i; //Coloca um valor em cada posição da matriz anteriores

    return anteriores; //Retorna a matriz anteriores
}

/**
 * @brief               Função auxiliar de Floyd, para iniciar a posição da matriz de distância contendo os pesos
 * @param   distancia   Vetor de distância a ser inicializado
 * @param   tam         Tamanho da matriz quadratica, recebe a ordem do grafo
 * 
 */
int **Graph::iniciaDistanciaFloyd(int **distancia, int tam)
{
    distancia = new int *[tam]; //Alocando a matriz distância
    for (int i = 0; i < tam; i++)
        distancia[i] = new int[tam]; //Aloca cada posição da matriz distância
    for (int i = 0; i < tam; i++)
        for (int j = 0; j < tam; j++)
            distancia[i][j] = INF; //Atribuindo a distância de infinito para todas a posições
    for (int i = 0; i < tam; i++)
        distancia[i][i] = 0; //Atribuindo a distância de 0 do vertice para ele mesmo

    Node *node = nullptr;
    Edge *edge = nullptr;

    for (int i = 0; i < tam; i++)
    { //Colocando o peso das arestas na matriz, caso o grafo não seja ponderado o valor 1 será atribuido para cada salto
        node = getNodePosition(i);
        edge = node->getFirstEdge();

        while (edge != nullptr)
        {

            if (!getWeightedEdge())
                distancia[i][edge->getTargetPosition()] = 1; //Adiciona 1 para cada salto caso o grafo não seja ponderado
            else
                distancia[i][edge->getTargetPosition()] = edge->getWeight(); //Adiciona o peso nessa posição da matriz
            edge = edge->getNextEdge();                                      //Avança a aresta
        }
    }
    return distancia; //Retorna a matriz distância
}

/**
 * @brief               Função auxiliar de floyd para imprimir e salvar em arquivo dot o caminho mínimo
 * @param   antecessor  Lista contendo os antecessores do caminho minimo, representa a ordem de acesso
 */
void Graph::saidaFloyd(int **pred, Node *noSource, Node *noTarget, ofstream &outFile)
{

    list<int> ordemAcesso; //Lista para conteer a ordem de acesso dos vertices, de trás para frente
    int ant;

    ordemAcesso.push_front(noTarget->getPosition());              //Adiciona a posição do no Target na filha
    ant = pred[noSource->getPosition()][noTarget->getPosition()]; //Pega a possição do antercessor ao no Source e Target

    while (ant != noSource->getPosition())
    { //Loop que adciona na lista todas as posições dos nos do menor caminho
        ordemAcesso.push_front(ant);
        ant = pred[noSource->getPosition()][ant];
    }
    ordemAcesso.push_front(noSource->getPosition());

    caminhoMinimo(ordemAcesso, outFile);
}

/**
 * @brief               Função auxiliar de disjkstra para imprimir o caminho
 * @param   antecessor  Vetor contendo o antecessor de cada posição
 * @param   idSource    ID do nó de origem
 * @param   idTarget    ID do nó de destino   
 */
void Graph::saidaDijkstra(int antecessor[], int idSource, int idTarget, ofstream &outFile)
{

    string arco;
    int noAnterior, primeiro, tNode, sNode; //Usado para armazenar o vertice anterior, e auxiliar na escrita no arquivo dot

    list<int> ordemAcesso; //Lista contendo a ordem de acesso dos vertices

    ordemAcesso.push_front(idTarget);  //Armazena na lista na ordem de acesso dos vertices,
    noAnterior = antecessor[idTarget]; //apartir de seus anteriores, começando pelo nó target

    while (noAnterior != idSource)
    {
        ordemAcesso.push_front(noAnterior);
        noAnterior = antecessor[noAnterior];
    }
    ordemAcesso.push_front(idSource); //Insere o nó Source como o primeiro a ser acessado
    primeiro = ordemAcesso.front();

    caminhoMinimo(ordemAcesso, outFile);
}

/**
 * @brief Verifica se o grafo é ciclico ou se é cíclico e direcionado
 * 
 * @return true 
 * @return false 
 */
bool Graph::thisIsCyclic()
{
    if (this->directed)
        return this->isCyclicDirected();
    else
        return this->isCyclic();
}

/**
 * @brief Função auxiliar para verificação se o grafo é cíclico
 * 
 * @param nodeId 
 * @param isVisited 
 * @param parentId 
 * @return true 
 * @return false 
 */
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
/**
 * @brief Função auxiliar para verificação se o graph é cíclico 
 * 
 * @return true 
 * @return false 
 */
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
/**
 * @brief Função auxiliar para verificação se o graph é cíclico e direcionado
 * 
 * @param nodeId 
 * @param isVisited 
 * @param isContainedRecusirve 
 * @return true 
 * @return false 
 */
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
/** 
 * @brief Função boleana que verifica se o graph é ciclíco
 * 
 * @return true 
 * @return false 
 */
bool Graph::isCyclicDirected()
{
    bool *isVisited = new bool[this->getOrder()];
    bool *isContainedRecusirve = new bool[this->getOrder()];
    Node *node = first_node;

    for (int i = 0; i < this->getOrder(); i++)
    {
        isVisited[i] = false; // Verifica se o nó atual foi visitado.
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
        node->setVisited(true); // Marca o nó atual como visitado.
        Node *auxNode;          // cria nó axiliar
        edge = node->getFirstEdge();

        while (edge != nullptr)
        {
            auxNode = this->getNode(edge->getTargetId()); // Aponta o no auxiliar para edge
            if (!auxNode->getVisited())
                topologicalSortUtil(auxNode, edge, Stack); // recursivo para todos o vértices adjacentes a ele

            edge = edge->getNextEdge();
        }

        if (node != nullptr) //Empilha o vértice atual
            Stack.push(node->getId());
    }
}

// 2a PARTE DO TRABALHO  ------------------------------------------------------------------------------------------------

/**
 * @brief                                   Função usada para imprimir a solução da AGM com restrição de grau
 * 
 * @param AGM_RESULTANTE 
 * @param posInicial 
 * @param alfa 
 */

void imprimeSolucao(vector<pair<int, pair<int, int>>> &AGM_RESULTANTE, int posInicial, float alfa){

    ofstream arq("meuGrafo.dot");               //Arquivo de saida
    ofstream saida("saida.txt", ios::out| ios::app);               //Arquivo de saida
    arq << "graph {\n";

    int soma = 0;                               //Soma dos pesos
    int qnt2 = 0;                               //Quantidade de arestas - 1, já AGM_RESULTANTE.size é a ordem da AGM - 1

    for(int i = 0;i<AGM_RESULTANTE.size(); i++){
        
            //É mostrado a posição do vertice no grafo é não seu ID
            arq << AGM_RESULTANTE[i].second.first << " -- " << AGM_RESULTANTE[i].second.second << "[ label = " << AGM_RESULTANTE[i].first << "]\n";
            saida <<"QUANTIDADE DE ARESTAS: " << qnt2+1 << " posição de u: " << AGM_RESULTANTE[i].second.first << " -- posição de v: " << AGM_RESULTANTE[i].second.second << " PESO DA ARESTA " << AGM_RESULTANTE[i].first << " ]\n";
            cout << qnt2 <<" QUANTIDADE DE ARESTAS: " << qnt2+1 << " posição de u: " << AGM_RESULTANTE[i].second.first << " -- posição de v: " << AGM_RESULTANTE[i].second.second << " PESO DA ARESTA " << AGM_RESULTANTE[i].first << " ]\n";

            soma += AGM_RESULTANTE[i].first;
            qnt2++;

    }

    arq << "}\n";
    saida << "SOMA DA AGM DE ORDEM "<< AGM_RESULTANTE.size()+1 <<" COMEÇANDO PELO VERTICE DE POSIÇÃO "<< posInicial<< " E COM O ALFA "<< alfa<< " : " << soma << endl;
    
    arq.close();
    saida.close();
    cout << "SOLUÇÃO SALVA NO ARQUIVO saida.dot e saida.txt"<<endl;
    

}
/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @return true 
 * @return false 
 */
bool maiorDasAretas(pair<int, pair<int, int>> x, pair<int, pair<int, int>> y){

    return x.first > y.first;
    
}

/**
 * @brief                                               Função usada para pegar todas as arestas de um determinado vertice apartir de sua posição
 * 
 * @param grafo 
 * @param posNo 
 * @param arestas 
 * @param pesoEGrauNo 
 * @param naAgm 
 */
void arestaDoNo(Graph *grafo, int posNo, vector<pair<int, pair<int, int>>> &arestas, vector<int> &pesoEGrauNo, vector<bool> &naAgm, bool visualizar){
    Node *noAux = grafo->getNodePosition(posNo);
    Edge *arestaAux = nullptr;

    if(visualizar)cout << "Pegando as arestas do vertice da posição "<< posNo <<" que estão dentro das retrições \n"<<  endl;
    
    int pos_u, pos_v;
    if(noAux == nullptr)
        return;

    else{
        pos_u = posNo;
        arestaAux = noAux->getFirstEdge();          //PEGA A PRIMEIRA ARESTA ADJACENTE AO VERTICE pos_u(POSISÃO EM QUE ELE É INSERIDO NO GRAFO, PELAS NOSSAS INSTÂNCIAS
                                                    //UM NÓ COM pos_u = 0 representa o ID = 1, pos_u = 1 representa o ID = 2, assim por diante)
    }
    
    while (arestaAux != nullptr){                   //REPETE ATÉ A ARESTA FOR NULL

        pos_v = arestaAux->getTargetPosition();

        //ADICIONA A ARESTA QUE LIGA O VERTICE DE POSIÇÃO pos_u ao vertice de POSIÇÃO pos_v CASO É ESTEJA NA CODIÇÃO DE INSERSÃO
        if (pesoEGrauNo[pos_u] < D && pesoEGrauNo[pos_v]< D && !naAgm[pos_v] && pos_v != pos_u)
            arestas.push_back({arestaAux->getWeight(), {pos_u, pos_v}});
        
        arestaAux = arestaAux->getNextEdge();       //Proxima aresta
    }
    

    int i = 0;
    while(i < arestas.size())
        if (naAgm[arestas[i].second.first] && naAgm[arestas[i].second.second])
           arestas.erase(arestas.begin() + i);
           
        else
            if (pesoEGrauNo[arestas[i].second.first] >= D || pesoEGrauNo[arestas[i].second.second] >= D)
                arestas.erase(arestas.begin() + i);
            else i++;

    sort(arestas.begin(), arestas.end());    //Organiza por peso as arestas
    
}

/**
 * @brief                                                   Função utilizada para criar a arvore geradora minima, com restrição de grau. com D = 3
 * 
 * @param grafo                                             
 * @param AGM_RESULTANTE 
 * @param melhor_solucao 
 * @param alfa 
 * @param posNoInicial 
 */
void geraAGM(Graph *grafo, vector<pair<int, pair<int, int>>> &AGM_RESULTANTE, int *melhor_solucao, float alfa, int *posNoInicial, int visualizar){
    
    unsigned seed = time(0);                                                    //Muda a seed dos aleatorios
    vector<pair<int, pair<int, int>>> arestas;                                  //vector<{peso, {pos_noOrigem, pos_noDestino}}>
    vector<int> pesoEGrauNo(grafo->getOrder(), 0);                              //CONTEM O GRAU DO VERTICE POS_U INSERIDO NA AGM
    vector<bool> naAGM(grafo->getOrder(), false);                               //MARCA SE O VERTICE pos_x ESTÁ NA AGM

    vector<pair<int, pair<int, int>>> AGM(grafo->getOrder());                   //VETOR QUE VAI GUARDA AS ARESTAS DA AGM pos_u -- pos_v ONDE 
                                                                                //pos_u e pos_v é a ordem que em esses vértices foram inseridos no grafo
    AGM.clear();

    int pos_u;
    int pos_v;
    int peso;
    int posInicial = 0;

    srand(seed);                                                                //Seed dos aleatorios
    posInicial = rand()%grafo->getOrder()-1;                                    //PEGA UM VERTICE ALEATORIO DO GRAU POR SUA POSIÇÃO DE INSERSÃO
    *posNoInicial = posInicial;

    arestaDoNo(grafo, posInicial, arestas,pesoEGrauNo, naAGM, visualizar);      //ADICIONA NO CONJUNTO DE SOLUÇÕES AS ARESTAS QUE PODEM ENTRAR NA AGM

    bool cond = true;
    int cont = 0;
    while ((find(naAGM.begin(), naAGM.end(), false) != naAGM.end()) && cond){   //ENQUANTO TODOS OS VERTICES NÃO ESTIVEREM NA AGM


            int k = 0;                                                          //A K-ESSIMA ARESTAS A SER INSERIDA, SE POSSIVEL, NO RANDOMIDO 
                                                                                //O K VARIA DE 0 A arestas.size * alfa
            if(arestas.size() > 0){
                                            
                if(int(arestas.size() * alfa) > 1)                              //O K VARIA DE 0 A arestas.size * alfa SE O ALFA FOR MAIOR QUE 0 E O VETOR DE ARESTA TIVER MAIS DE 2 CONTIDOS
                    k = rand()%int(arestas.size() * alfa);                  
                
                pos_u = arestas[k].second.first;                                //VERTICE COM POSISÃO DE INSERÇÃO pos_u
                pos_v = arestas[k].second.second;                               //VERTICE COM POSISÃO DE INSERÇÃO pos_v
                peso = arestas[k].first;                                        //PESO DA ARESTA pos_u pos_v
        
                if((naAGM[pos_u] && naAGM[pos_v]) || (pesoEGrauNo[pos_u] >= D || pesoEGrauNo[pos_v] >= D)){

                    arestas.erase(arestas.begin() + k);                         //REMOVE A ARESTA SE O GRAU DO pos_u ou pos_v for maior que D,
                                                                                //ALÉM DE REMOVE-LO SE pos_u e pos_v JÁ CONSTAR NA AGM

                }else{
                    if(pos_u != pos_v){                                         //GARANTE QUE NÃO TENHA SELF LOOP
                        
                        AGM.push_back({peso, {pos_u,pos_v}});                   //ADICIONA NA SOLUÇÃO A ARESTA pos_u pos_v
                        naAGM[pos_u] = true;                                    //MARCA COM JÁ ADICIONADO
                        naAGM[pos_v] = true;                                    //MARCA COM JÁ ADICIONADO
                        pesoEGrauNo[pos_u] += 1;                                //AUMENTA SEU GRAU
                        pesoEGrauNo[pos_v] += 1;                                //AUMENTA SEU GRAU
                        arestas.erase(arestas.begin() + k);                     //REMOVE DO VERTOR DE POSIVEIS SOLUÇÕES

                        if(visualizar)cout << "Vértice de poição "<< pos_v << " adicionado a AGM" << endl;

                        arestaDoNo(grafo, pos_v, arestas,pesoEGrauNo, naAGM, visualizar); //ADICIONA TODAS A ARESTAS DO NO RECEM ADICIONADO NO VETOR DE POSSIVEIS SOLUÇÕES 
                    }
                }
            } else{
                cout << "CONJUNTO DE POSSIVEIS SOLUÇÕES VAZIA"<<endl;
                cond = false;
            }
            
    }
    int soma = 0;
    for(int i = 0;i<AGM.size(); i++)                        //SOMA O PESO DA AGM
        if(AGM[i].second.first != AGM[i].second.second)
            soma += AGM[i].first;
        
    *melhor_solucao = soma;                                 //ATUALIZA O VALOR DO PONTEIRO
    AGM_RESULTANTE = AGM;                                   //ATUALIZA O VALOR DO PONTEIRO

}

/**
 * @brief Função que recebe dois pares como parametro e retorna se o first de i for maior
 * 
 * @param i par
 * @param j par
 * @return true 
 * @return false 
 */
bool maior(pair<int, int> i, pair<int, int> j)
{
    return (i.first > j.first);
}

/**
 * @brief 
 * 
 * @param vet 
 * @param val 
 * @return int 
 */
int acha(vector<pair<int, int>> &vet, pair<int, int> val)
{
    int i = 0;
    for (i = 0; i < vet.size(); i++)
    {
        if (vet[i].first == val.first && vet[i].second == val.second)
            return i;
    }
    return INF;
}


/* A função ordenaArestas recebe como parâmetro uma matriz e uma opção. Na matriz,
    o vector de fora representa cada nó e, para cada nó, há um vector de pares, em que
    cada par representa uma aresta daquele nó. O par tem o formato {peso, posicao_notarget}.
    A função ordena cada uma dessas arestas de cada nó. Se op = 0, ordem crescente. Se op = 1, descrescente*/
void ordenaArestas(vector<vector<pair<int, int>>> &matriz, int op)
{
    for (int i = 0; i < matriz.size(); i++)
    {
        if (op == 0)
            sort(matriz[i].begin(), matriz[i].end());
        else if (op == 1)
            sort(matriz[i].begin(), matriz[i].end(), maior);
    }
}

/* A função ordenaArestas recebe como parâmetro uma matriz em que o vector de fora
    representa cada nó e, para cada nó, há um vector de pares, em que
    cada par representa uma aresta daquele nó. O par tem o formato {peso, posicao_notarget}.
    A função retorna essa matriz preenchida*/
void preencheMatriz(Graph *grafo, vector<vector<pair<int, int>>> &matriz)
{
    grafo->cleanVisited();
    Node *noAux = grafo->getFirstNode();
    Edge *arestaAux = noAux->getFirstEdge();

    int pos_u = noAux->getPosition(); // id do nó de origem
    int v;

    if (arestaAux != nullptr)
        v = arestaAux->getTargetId(); //id do nó destino

    // Percorrer todas as arestas do Grafo
    for (int i = 1; i < grafo->getOrder(); i++)
    {
        if (arestaAux == nullptr)
            matriz[pos_u].push_back({INF, pos_u});

        while (arestaAux != nullptr)
        {
            // Coloca a aresta na matriz de acordo com o nó de origem correspondente
            if (!grafo->getNode(v)->getVisited())
            {
                int pos_v = grafo->getNode(v)->getPosition();
                matriz[pos_u].push_back({arestaAux->getWeight(), pos_v});
            }

            // Atualiza os auxiliares se a aresta não for null
            arestaAux = arestaAux->getNextEdge();
            if (arestaAux != nullptr)
                v = arestaAux->getTargetId();
        }

        noAux->setVisited(true);
        noAux = grafo->getNodePosition(i);
        arestaAux = noAux->getFirstEdge();
        pos_u = noAux->getPosition();
        if (arestaAux != nullptr)
            v = arestaAux->getTargetId();
    }
}

void preencheMatrizAGM(Graph *grafo, vector<int> &agm, vector<pair<int, pair<int, int>>> &arestas, vector<vector<pair<int, int>>> &matriz, vector<vector<pair<int, int>>> &matrizAGM, vector<pair<int, int>> &pesoEGrauNo, vector<bool> &dentroRestricao)
{
    for (int i = 0; i < agm.size(); i++)
    {
        //cout << "i = " << i << " tem-se: {" << matriz[i][0].first << ", " << matriz[i][0].second << "}" << endl;
        if (agm[i] != INF)
        {
            // arestas = {peso, {pos_noOrigem, pos_noDest}}
            int pos_u = arestas[agm[i]].second.first;
            int pos_v = arestas[agm[i]].second.second;
            int peso = arestas[agm[i]].first;

            int grau;

            pair<int, int> aresta(peso, pos_u);
            //cout << "aresta: {" << aresta.first << ", " << aresta.second << endl;
            int it = acha(matriz[pos_v], aresta);

            if (it != INF)
            {
                //cout << " . ";
                matrizAGM[pos_v].push_back(aresta);              // insere a aresta na matrizAGM
                matriz[pos_v].erase(matriz[pos_v].begin() + it); // apaga da matriz
            }
            else
            {
                aresta = {peso, pos_v};
                it = acha(matriz[pos_u], aresta);
                if (it != INF)
                {
                    //cout << " ! ";
                    matrizAGM[pos_u].push_back(aresta);              // insere a aresta na matrizAGM
                    matriz[pos_u].erase(matriz[pos_u].begin() + it); // apaga da matriz
                }
            }

            pesoEGrauNo[pos_u].second++;
            pesoEGrauNo[pos_v].second++;
            if (pesoEGrauNo[pos_u].second > D)
            {
                dentroRestricao[pos_u] = false;
                cout << "\nID do nó fora da restrição: " << grafo->getNodePosition(pos_u)->getId() << endl;
                cout << "Posição do nó fora da restrição: " << pos_u << endl;
                cout << "Grau do nó fora da restrição: " << pesoEGrauNo[pos_u].second << endl;
            }
            if (pesoEGrauNo[pos_v].second > D)
            {
                dentroRestricao[pos_v] = false;
                cout << "\nID do nó fora da restrição: " << grafo->getNodePosition(pos_v)->getId() << endl;
                cout << "Posição do nó fora da restrição: " << pos_v << endl;
                cout << "Grau do nó fora da restrição: " << pesoEGrauNo[pos_v].second << endl;
            }
            pesoEGrauNo[pos_u].first += peso;
            pesoEGrauNo[pos_v].first += peso;
        }
    }
}
void kruskalAdaptado(Graph *grafo, vector<vector<pair<int, int>>> &matriz, vector<vector<pair<int, int>>> &matrizAGM, vector<pair<int, int>> &pesoEGrauNo, vector<bool> &dentroRestricao,int *pesoFinal, vector<vector<pair<int, int>>> &matrizAGMFINAL,float alfa = 0)
{

    vector<pair<int, pair<int, int>>> arestas; //vector<{peso, {pos_noOrigem, pos_noDestino}}>
    arestas.clear();

    grafo->cleanVisited();
    Node *noAux = grafo->getFirstNode();
    Edge *arestaAux = noAux->getFirstEdge();

    int u = noAux->getId(); // id do nó de origem
    int pos_u = noAux->getPosition();
    int v;
    int pos_v;

    if (arestaAux != nullptr)
    {
        v = arestaAux->getTargetId(); //id do nó destino
        pos_v = grafo->getNode(v)->getPosition();
    }
    // Percorrer todas as arestas do Grafo
    for (int i = 1; i < grafo->getOrder(); i++)
    {
        if (arestaAux == nullptr)
            arestas.push_back({INF, {pos_u, pos_u}});

        while (arestaAux != nullptr)
        {
            // Coloca a aresta no vetor de arestas
            if (!grafo->getNode(v)->getVisited())
                arestas.push_back({arestaAux->getWeight(), {pos_u, pos_v}});

            // Atualiza os auxiliares se a aresta não for null
            arestaAux = arestaAux->getNextEdge();
            if (arestaAux != nullptr)
            {
                v = arestaAux->getTargetId();
                pos_v = grafo->getNode(v)->getPosition();
            }
        }

        noAux->setVisited(true);
        noAux = grafo->getNodePosition(i);
        arestaAux = noAux->getFirstEdge();
        u = noAux->getId();
        pos_u = noAux->getPosition();
        if (arestaAux != nullptr)
        {
            v = arestaAux->getTargetId();
            pos_v = grafo->getNode(v)->getPosition();
        }
    }

    // Ordena do menor peso para o maior
    sort(arestas.begin(), arestas.end()); 

    int ordem = grafo->getOrder();
    SubArvore *subarvores = new SubArvore[(ordem * sizeof(SubArvore))]; // vetor para armazenar todas as subárvores

    for (int i = 0; i < ordem; i++)
    {
        subarvores[i].pai = i;
        subarvores[i].ordem = 1;
    }

    //Montar a Árvore Geradora Mínima

    vector<int> agm; // vetor com o índice associado à posição de cada aresta da árvore geradora mínima no vector 'arestas' do subgrafo
    agm.clear();

    // Iterar até atingir condição de parada
    int k = 0;
    vector<pair<int, pair<int, int>>> arestasAux;
    arestasAux.clear();
    arestasAux = arestas;
    vector<int>grau(grafo->getOrder(),0);
    int cont=0;
    while (arestas.size() > 0)
    {   
        k=randAlfa(alfa, arestas.size());
        pair<int, int> proxAresta = arestas[k].second;
        
        int pos_u = proxAresta.first;
        int pos_v = proxAresta.second;

        // Se u e v não estão na mesma subárvore
        if(grau[pos_u] < D && grau[pos_v] < D)
        if (qualSubArvore(subarvores, pos_u) != qualSubArvore(subarvores, pos_v))
        {
            agm.push_back(cont+k);
            unirSubArvores(subarvores, pos_u, pos_v);
            grau[pos_u] += 1;
            grau[pos_v] += 1;
            
        }
        cont++;
        arestas.erase(arestas.begin());
        
    }

    preencheMatrizAGM(grafo, agm, arestasAux, matriz, matrizAGM, pesoEGrauNo, dentroRestricao);

    *pesoFinal=pesoTotalAGM(pesoEGrauNo);
    matrizAGMFINAL = matrizAGM;

    delete[] subarvores;
    return;
}

int pesoTotalAGM(vector<pair<int, int>> &pesoNoTotal)
{
    int peso = 0;
    for (int i = 0; i < pesoNoTotal.size(); i++)
        peso += pesoNoTotal[i].first;
    return peso / 2;
}

/**
 * @brief Algoritmo Guloso
 * 
 * @return float 
 */
void Graph::greed()
{
  
    /* matriz em que o vector de fora representa cada nó e,
    para cada nó, há um vector de pares, em que cada par representa
    uma aresta daquele nó. O par tem o formato {peso, posicao_notarget} */
    vector<vector<pair<int, int>>> matriz(this->getOrder());

    chrono::time_point<chrono::system_clock> inicio, fim;
    iniciaProces(&inicio);

    // Preenchendo e ordenando a matriz
    preencheMatriz(this, matriz);
    ordenaArestas(matriz, 1);

    //imprimirMatriz(matriz);

    // controles
    vector<pair<int, int>> pesoEGrauNo(this->getOrder(), {0, 0}); // par {peso, grau}
    vector<bool> dentroRestricao(this->getOrder(), true);         // vector para controlar os nós que estão dentro da restrição

    // matriz que vai ter as arestas da agm
    vector<vector<pair<int, int>>> matrizAGM(this->getOrder());

    int melhorSolucao2 = INF;
    vector<vector<pair<int, int>>> melhorMatrizAGM2;
    melhorMatrizAGM2.clear();

    kruskalAdaptado(this, matriz, matrizAGM, pesoEGrauNo, dentroRestricao, &melhorSolucao2, melhorMatrizAGM2);
    ordenaArestas(matrizAGM, 0);

    int pesoAGM = pesoTotalAGM(pesoEGrauNo);
    imprimirMatriz(matrizAGM);
    
    double temp = fimProces(&inicio, &fim);

    cout << "\ndepois:" << endl;
    // imprimirMatriz(matrizAGM);

    cout << "\nPeso da Arvore Geradora com Restrição de Grau: " << melhorSolucao2 << "\nTempo total: " << temp << " segundos" << endl;
    return;
}

void Graph::greedRandom(){


    float alfa;
    cout << "QUAL O ALFA"<<endl;
    cin >> alfa;
    int k_iteracoes = 1;
    cout << "QUANTIDADE DE ITERAÇÕES: "<<endl;
    cin >> k_iteracoes;
    cout << "\nINICIANDO A EXECUÇÃO DO ALGORITMO GULOSO RANDOMIZADO ..." << endl;
    
    chrono::time_point<chrono::system_clock> inicio, fim;
    iniciaProces(&inicio);
    int melhorSolucao1 = 0;
    int melhorSolucao2 = INF;

    vector<vector<pair<int, int>>> melhorMatrizAGM1;
    melhorMatrizAGM1.clear();
    vector<vector<pair<int, int>>> melhorMatrizAGM2;
    melhorMatrizAGM2.clear();

    for(int i = 0;i<k_iteracoes;i++){

        /* matriz em que o vector de fora representa cada nó e,
        para cada nó, há um vector de pares, em que cada par representa
        uma aresta daquele nó. O par tem o formato {peso, posicao_notarget} */
        vector<vector<pair<int, int>>> matriz(this->getOrder());


        // Preenchendo e ordenando a matriz
        preencheMatriz(this, matriz);
        ordenaArestas(matriz, 1);

        //imprimirMatriz(matriz);

        // controles
        vector<pair<int, int>> pesoEGrauNo(this->getOrder(), {0, 0}); // par {peso, grau}
        vector<bool> dentroRestricao(this->getOrder(), true);         // vector para controlar os nós que estão dentro da restrição

        // matriz que vai ter as arestas da agm
        vector<vector<pair<int, int>>> matrizAGM(this->getOrder());

        kruskalAdaptado(this, matriz, matrizAGM, pesoEGrauNo, dentroRestricao, &melhorSolucao1, melhorMatrizAGM1, alfa);
        ordenaArestas(matrizAGM, 0);

        if(melhorSolucao1 < melhorSolucao2){
            
            melhorMatrizAGM2 = melhorMatrizAGM1;
            melhorSolucao2 = melhorSolucao1;

        }
        cout << "MELHOR SOLUÇÃO: "<< melhorSolucao2<<endl;
        
    }
    double temp = fimProces(&inicio, &fim);
    imprimirMatriz(melhorMatrizAGM2);
    cout << "\nPeso da Arvore Geradora com Restrição de Grau: " << melhorSolucao2 << "\nTempo total: " << temp << " segundos" << endl;
}
void Graph::greedRactiveRandom(){

    int d;         // Grau de restrição;
    int numIt;     // Número de iterações;
    int numBloco;  // Número de iterações até atualizar novamente o vetor de probabilidades;
    int numAlfas;  // Variável ustilizada para armazenar a quantidade de alfas que serão analizados;
    int index;     // índice do alfa escolhido para a iteração;
    float solucao;   // Armazena a solução da iteração;
    float auxAlfa; // Variável auxiliar utilizada para armazenar o valor do alfa;

    cout << "\nQual será o número total de iterações?" << endl;
    cin >> numIt;
    cout << endl;

    cout << "\nQual será a quantidade de iterações por bloco?" << endl;
    cin >> numBloco;
    cout << endl;

    cout << "\nINICIANDO A EXECUÇÃO DO ALGORITMO GULOSO RANDOMIZADO REATIVO ..." << endl;
    chrono::time_point<chrono::system_clock> inicio, fim;

    iniciaProces(&inicio);
    vector<float> alfas;   // Vetor utilizado para armazenar os valores de alfa;
    vector<float> solBest;   // Vetor utilizado para armazenar melhor solução encontrada por cada alfa (Os valores são referentes ao alfa que se encontra no elemento de mesmo índice do vetor de alfas);
    vector<media> medias;  // Vetor utilizado para armazenar a média das soluções de cada alfa (Os valores são referentes ao alfa que se encontra no elemento de mesmo índice do vetor de alfas);
    vector<float> prob;    // Vetor utilizado para armazenar a probabilidade de cada alfa (Os valores são referentes ao alfa que se encontra no elemento de mesmo índice do vetor de alfas);;
    vector<float> q;       // Vetor utilizado para armazenar o q (operação necessária para o cálculo da probabilidade) de cada alfa (Os valores são referentes ao alfa que se encontra no elemento de mesmo índice do vetor de alfas);;

    alfas.push_back(0.05);
    alfas.push_back(0.10);
    alfas.push_back(0.15);
    alfas.push_back(0.30);
    alfas.push_back(0.50);

    vector<vector<pair<int, int>>> melhorMatrizAGM1;
    melhorMatrizAGM1.clear();
    
    numAlfas = alfas.size();

    cout << "numAlfas: " << numAlfas << endl;

    for(int i=0; i<numAlfas; i++){
        solBest.push_back(0.0);
    }

    for(int i=0; i<numAlfas; i++){
        q.push_back(0.0);
    }

    inicializaVetores(prob, medias, numAlfas); // Chamada da função para inicializar os vetores de médias e probabilidades;

    cout << "\nMedias: " << endl;
    for(int i=0; i< medias.size(); i++){
        cout << medias[i].media << " | ";
    }

    cout << "\nProbs: " << endl;
    for(int i=0; i< prob.size(); i++){
        cout << prob[i] << " | ";
    }

    vector<pair<int, pair<int, int>>> AGM_RESULTANTE1;
    vector<pair<int, pair<int, int>>> AGM_RESULTANTE2;
    int melhorSolucao1= 0;
    int melhorSolucao2= INF;
    int posNoInicial;

    int i = 0;
    while (i <= numIt){
        if (i!=0 && i % numBloco == 0)
        {
            cout << "\nProbs: " << endl;
            for(int i=0; i< prob.size(); i++){
                cout << prob[i] << " | ";
            }
            cout << endl;
            atualizaProbabilidades(medias, prob, solBest, q); // Chamada da função que irá atualizar o vetor de probabilidades;
            cout << "\nProbs: " << endl;
            for(int i=0; i< prob.size(); i++){
                cout << prob[i] << " | ";
            }
            cout << endl;
        }

        int index = escolheAlfa(prob);
        cout << "\nindexAlfa: " << index << endl;
        auxAlfa = alfas[index];
        cout << "auxAlfa: " << auxAlfa << endl;


        /* matriz em que o vector de fora representa cada nó e,
        para cada nó, há um vector de pares, em que cada par representa
        uma aresta daquele nó. O par tem o formato {peso, posicao_notarget} */
        vector<vector<pair<int, int>>> matriz(this->getOrder());
        matriz.clear();

        // Preenchendo e ordenando a matriz
        preencheMatriz(this, matriz);
        ordenaArestas(matriz, 1);

        //imprimirMatriz(matriz);

        // controles
        vector<pair<int, int>> pesoEGrauNo(this->getOrder(), {0, 0}); // par {peso, grau}
        vector<bool> dentroRestricao(this->getOrder(), true);         // vector para controlar os nós que estão dentro da restrição

        // matriz que vai ter as arestas da agm
        vector<vector<pair<int, int>>> matrizAGM(this->getOrder());
        
        kruskalAdaptado(this, matriz, matrizAGM, pesoEGrauNo, dentroRestricao, &melhorSolucao1, melhorMatrizAGM1, auxAlfa);
        ordenaArestas(matrizAGM, 0);

        cout<< i<<" melhorSolucao2 " << melhorSolucao1 << endl;

        atualizaMedias(medias, melhorSolucao1, alfas, auxAlfa);


        cout << "\nMedias: " << endl;
        for(int i=0; i< medias.size(); i++){
            cout << medias[i].media << " | ";
        }
        cout << endl;

        if(solBest[index] == 0){
            solBest[index] = melhorSolucao1;
        }

        if(solBest[index] > melhorSolucao1){
            solBest[index] = melhorSolucao1;
        }

        AGM_RESULTANTE1.clear();
        AGM_RESULTANTE2.clear();
        int melhorSolucao1= 0;
        int melhorSolucao2= INF;
        i++;
    }

    for(int i=0; i<solBest.size(); i++){
        cout << solBest[i] << " | ";
    }
    cout << endl;

    int auxSolBest = INF;
    for(int i=0; i<solBest.size(); i++){
        if(solBest[i] < auxSolBest && solBest[i]!=0){
            auxSolBest = solBest[i];
        }
    }
  double temp = fimProces(&inicio, &fim);
    cout << "\nMelhor Peso da Arvore Geradora com Restrição de Grau = " << auxSolBest
         << "\nTempo total: " << temp << " segundos" << endl;

}

/**
 * @brief 
 * 
 */
void Graph::greed2(){

    float alfa = 0;
    
    vector<pair<int, pair<int, int>>> AGM_RESULTANTE1;
    vector<pair<int, pair<int, int>>> AGM_RESULTANTE2;
    int melhorSolucao1= 0;
    int melhorSolucao2= INF;
    int posNoInicial;

    cout << "GOSTARIA DE VER O PROGRESSO DE INSERSÃO OCORRER(REQUER MAIS TEMPO)?\n1 PARA SIM\n2 PARA NÃO"<<endl;
    string querVisuzalizar;int visuzalizar = 0;
    cin >> querVisuzalizar;
    if(querVisuzalizar == "1")visuzalizar = 1;
    else
        if(querVisuzalizar == "2")visuzalizar = 0;
        else {cout << "OPÇÃO INVALIDA, 2 SELECIONADO AUTOMATICAMENTE"<<endl;visuzalizar = 0;}
    cout << "\nINICIANDO A EXECUÇÃO DO ALGORITMO GULOSO 2 ..." << endl;

    chrono::time_point<chrono::system_clock> inicio, fim;
    iniciaProces(&inicio);
    
    for(int i = 0;i<1;i++){

        geraAGM(this, AGM_RESULTANTE1,&melhorSolucao1, alfa, &posNoInicial, visuzalizar);
        cout<< i<<" MELHOR SOLUCÃO " << melhorSolucao1 << endl; 
        if(melhorSolucao1 < melhorSolucao2){
            
            melhorSolucao2 = melhorSolucao1;
            AGM_RESULTANTE2 = AGM_RESULTANTE1;

        }
        
    }
    double temp = fimProces(&inicio, &fim);
    imprimeSolucao(AGM_RESULTANTE2, posNoInicial, alfa);
    cout << "SOLUÇÃO FINAL: " <<melhorSolucao2
         << "\nTempo total: " << temp << " segundos" << endl;

}
/**
 * @brief 
 * 
 */
void Graph::greedRandom2(){

    float alfa;
    cout << "QUAL O ALFA"<<endl;
    cin >> alfa;

    cout << "GOSTARIA DE VER O PROGRESSO DE INSERSÃO OCORRER(REQUER MAIS TEMPO)?\n1 PARA SIM\n2 PARA NÃO"<<endl;
    string querVisuzalizar;int visuzalizar = 0;
    cin >> querVisuzalizar;
    if(querVisuzalizar == "1")visuzalizar = 1;
    else
        if(querVisuzalizar == "2")visuzalizar = 0;
        else {cout << "OPÇÃO INVALIDA, 2 SELECIONADO AUTOMATICAMENTE"<<endl;visuzalizar = 0;}
    cout << "\nINICIANDO A EXECUÇÃO DO ALGORITMO GULOSO RANDOMIZADO 2 ..." << endl;
    chrono::time_point<chrono::system_clock> inicio, fim;
    iniciaProces(&inicio);

    vector<pair<int, pair<int, int>>> AGM_RESULTANTE1;
    vector<pair<int, pair<int, int>>> AGM_RESULTANTE2;
    int melhorSolucao1= 0;
    int melhorSolucao2= INF;
    int posNoInicial;
    for(int i = 0;i<1;i++){

        geraAGM(this, AGM_RESULTANTE1,&melhorSolucao1, alfa, &posNoInicial, visuzalizar);
        cout<< i<<" MELHOR SOLUÇÃO " << melhorSolucao1 << endl; 
        if(melhorSolucao1 < melhorSolucao2){
            
            melhorSolucao2 = melhorSolucao1;
            AGM_RESULTANTE2 = AGM_RESULTANTE1;

        }
        
    }
    double temp = fimProces(&inicio, &fim);
    imprimeSolucao(AGM_RESULTANTE2, posNoInicial, alfa);
    cout << "SOLUÇÃO FINAL " <<melhorSolucao2
        << "\nTempo total: " << temp << " segundos" << endl;

}
/**
 * @brief 
 * 
 */
void Graph::greedRactiveRandom2(){

    int numIt;     // Número de iterações;
    int numBloco;  // Número de iterações até atualizar novamente o vetor de probabilidades;
    int numAlfas;  // Variável ustilizada para armazenar a quantidade de alfas que serão analizados;
    int index;     // índice do alfa escolhido para a iteração;
    float solucao;   // Armazena a solução da iteração;
    float auxAlfa; // Variável auxiliar utilizada para armazenar o valor do alfa;

    cout << "GOSTARIA DE VER O PROGRESSO DE INSERSÃO OCORRER(REQUER MAIS TEMPO)?\n1 PARA SIM\n2 PARA NÃO"<<endl;
    string querVisuzalizar;int visuzalizar = 0;
    cin >> querVisuzalizar;
    if(querVisuzalizar == "1")visuzalizar = 1;
    else
        if(querVisuzalizar == "2")visuzalizar = 0;
        else {cout << "OPÇÃO INVALIDA, 2 SELECIONADO AUTOMATICAMENTE"<<endl;visuzalizar = 0;}

    cout << "\nQual será o número total de iterações?" << endl;
    cin >> numIt;
    cout << endl;

    cout << "\nQual será a quantidade de iterações por bloco?" << endl;
    cin >> numBloco;
    cout << endl;

    cout << "\nINICIANDO A EXECUÇÃO DO ALGORITMO GULOSO RANDOMIZADO REATIVO 2..." << endl;
        chrono::time_point<chrono::system_clock> inicio, fim;
    iniciaProces(&inicio);
    vector<float> alfas;   // Vetor utilizado para armazenar os valores de alfa;
    vector<float> solBest;   // Vetor utilizado para armazenar melhor solução encontrada por cada alfa (Os valores são referentes ao alfa que se encontra no elemento de mesmo índice do vetor de alfas);
    vector<media> medias;  // Vetor utilizado para armazenar a média das soluções de cada alfa (Os valores são referentes ao alfa que se encontra no elemento de mesmo índice do vetor de alfas);
    vector<float> prob;    // Vetor utilizado para armazenar a probabilidade de cada alfa (Os valores são referentes ao alfa que se encontra no elemento de mesmo índice do vetor de alfas);;
    vector<float> q;       // Vetor utilizado para armazenar o q (operação necessária para o cálculo da probabilidade) de cada alfa (Os valores são referentes ao alfa que se encontra no elemento de mesmo índice do vetor de alfas);;

    alfas.push_back(0.05);
    alfas.push_back(0.10);
    alfas.push_back(0.15);
    alfas.push_back(0.30);
    alfas.push_back(0.50);

    numAlfas = alfas.size();

    cout << "numAlfas: " << numAlfas << endl;

    for(int i=0; i<numAlfas; i++){
        solBest.push_back(0.0);
    }

    for(int i=0; i<numAlfas; i++){
        q.push_back(0.0);
    }

    inicializaVetores(prob, medias, numAlfas); // Chamada da função para inicializar os vetores de médias e probabilidades;

    cout << "\nMedias: " << endl;
    for(int i=0; i< medias.size(); i++){
        cout << medias[i].media << " | ";
    }

    cout << "\nProbs: " << endl;
    for(int i=0; i< prob.size(); i++){
        cout << prob[i] << " | ";
    }

    vector<pair<int, pair<int, int>>> AGM_RESULTANTE1;
    vector<pair<int, pair<int, int>>> AGM_RESULTANTE2;
    int melhorSolucao1= 0;
    int melhorSolucao2= INF;
    int posNoInicial;

    int i = 0;
    while (i <= numIt){
        if (i!=0 && i % numBloco == 0)
        {
            cout << "\nProbs: " << endl;
            for(int i=0; i< prob.size(); i++){
                cout << prob[i] << " | ";
            }
            cout << endl;
            atualizaProbabilidades(medias, prob, solBest, q); // Chamada da função que irá atualizar o vetor de probabilidades;
            cout << "\nProbs: " << endl;
            for(int i=0; i< prob.size(); i++){
                cout << prob[i] << " | ";
            }
            cout << endl;
        }

        int index = escolheAlfa(prob);
        cout << "\nindexAlfa: " << index << endl;
        auxAlfa = alfas[index];
        cout << "auxAlfa: " << auxAlfa << endl;

        geraAGM(this, AGM_RESULTANTE1,&melhorSolucao1, auxAlfa, &posNoInicial, visuzalizar);
        cout<< i<<" melhorSolucao2 " << melhorSolucao1 << endl;

        atualizaMedias(medias, melhorSolucao1, alfas, auxAlfa);

        cout << "\nMedias: " << endl;
        for(int i=0; i< medias.size(); i++){
            cout << medias[i].media << " | ";
        }
        cout << endl;

        if(solBest[index] == 0){
            solBest[index] = melhorSolucao1;
        }

        if(solBest[index] > melhorSolucao1){
            solBest[index] = melhorSolucao1;
        }

        AGM_RESULTANTE1.clear();
        AGM_RESULTANTE2.clear();
        int melhorSolucao1= 0;
        int melhorSolucao2= INF;
        i++;
    }

    for(int i=0; i<solBest.size(); i++){
        cout << solBest[i] << " | ";
    }
    cout << endl;

    int auxSolBest = INF;
    for(int i=0; i<solBest.size(); i++){
        if(solBest[i] < auxSolBest && solBest[i]!=0){
            auxSolBest = solBest[i];
        }
    }
  double temp = fimProces(&inicio, &fim);
    cout << "\nMelhor Peso da Arvore Geradora com Restrição de Grau = " << auxSolBest
         << "\nTempo total: " << temp << " segundos" << endl;
    
}


//TODO: Funções auxiliares para os algorítimos gulósos randomizados -------------------------------------------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @param matriz 
 */
void imprimirMatriz(vector<vector<pair<int, int>>> &matriz)
{
    ofstream arq("saida.dot");

    arq << "graph {\n";
    int sizeAll = 0;
    int soma = 0;
    int qnt2 = 0;
    for (int i = 0; i < matriz.size(); i++)
    {
        for (int j = 0; j < matriz[i].size(); j++)
        {

            arq << i << " -- " << matriz[i][j].second << "[ label = " << matriz[i][j].first << "]\n";
            cout << qnt2 << " " << i << " -- " << matriz[i][j].second << "[ label = " << matriz[i][j].first << " ]\n";
            soma += matriz[i][j].first;
            qnt2++;
        }
    }
    arq << "}\n";
    cout << "SOMA: " << soma << endl;
    arq.close();
}
/**
 * @brief 
 * 
 * @param alfa 
 * @param tam_vetor 
 * @return int 
 */
int randAlfa(float alfa, int tam_vetor)
{
    float k;
    int aux = int(tam_vetor * alfa);
    if (alfa > 0.0 && aux > 0)
        k = (rand() % ((int)(alfa * tam_vetor)));
    else
        k = 0.0;
    return (int)k;
}

/**
 * @brief           Operação responsável por inicializar os vetores associados aos valores alfa
 * 
 * @param prob      Vetor das probabilidades de alfa
 * @param media     Vetor das médias das soluções de cada alfa
 * @param numAlfas  Número de alfas existentes
 */
void inicializaVetores(vector<float> &prob, vector<media> &medias, int &numAlfas)
{

    media aux; // Variável do tipo media utilizada para iniciar todos os elementos do vetor de médias com valor 0;
    aux.soma = 0;
    aux.numSolucoes = 0;
    aux.media = 1;
    float auxNumAlfas = numAlfas;

    float auxProb = 1/auxNumAlfas;
    cout << "\nauxProb: " << auxProb << endl;

    for (int i = 0; i < numAlfas; i++)
    {
        prob.push_back(auxProb); // Os alfas começam com a mesma probabilidade;
        medias.push_back(aux);        // Como não foi encontrada nenhuma solução, todas as médias começam com valor 0;
    }
}

/**
 * @brief           Operação utilizada para recalcular as probabilidades dos alfas de serem escolhidos
 * 
 * @param A         Vetor das médias das soluções encontradas por cada alfa
 * @param prob      Vetor que armazena a probabilidade de cada alfa ser escolhido
 * @param solBest   Vetor que armazena a melhor solução encontrada por cada alfa
 * @param q         Vetor que armazena o valor "q" (resultado de uma operação necessária para calcular a probabilidade) de cada alfa
 */
void atualizaProbabilidades(vector<media> &medias, vector<float> &prob, vector<float> &solBest, vector<float> &q)
{

    float somaQ = 0; // Variável criada para armazenar a soma de todos os valores "q";

    for (int i = 0; i < medias.size(); i++)
    {
        q[i] = pow((solBest[i] / medias[i].media), 100); // q[i] receberá o valor da divisão da melhor solução do respectivo alfa pela média de suas soluções elevado a um valor de precisão;
        somaQ = somaQ + q[i];                                 // À medida que se calcula os valores de "q", tal valor é incrementado em somaQ;
    }

    //cout << "ERRO" << endl;

    for (int i = 0; i < medias.size(); i++)
    {
        prob[i] = q[i] / somaQ; // Operação para calcular a probabilidade de cada alfa dada pela divisão de q[i] pela soma de todos os valores "q";
    }
}

void atualizaMedias(vector<media> &medias, int solucao, vector<float> &alfas, float alfa)
{

    int aux;           // Variável utilizada para armazenar o índice referente à posição do alfa no vetor de alfas;
    float auxSolucao;  // Variavel auxiliar para transformar a solução em float;

    auxSolucao = solucao;
 
    cout << "Solucao: " << auxSolucao << endl;

    for (int i = 0; i < alfas.size(); i++)
    {
        if (alfa == alfas[i])
        {
            aux = i; // Atribuindo o índice correto a aux;
            break;
        }
    }

    cout << "AuxIndice: " << aux << endl;

    medias[aux].soma = medias[aux].soma + auxSolucao;                  //  Atualizando o valor da soma;
    medias[aux].numSolucoes++;                                      //  Atualizando o número de soluções;
    medias[aux].media = medias[aux].soma / medias[aux].numSolucoes; //  Recalculando a média do respectivo alfa utilizando os valores atualizados da soma e numero de soluções;
}

/**
 * @brief   Função auxiliar para escolher o alfa de acordo com o vetor de probabilidades
 * 
 * @param   prob  Vetor de probabilidades
 * @return  int  Índice do alfa escolhido
 */
int escolheAlfa(vector<float> &prob)
{
    
    int index;       // Inteiro utilizado para armazenar o índice do alfa escolhido;
    int auxIndex;    // Auxiliar para armazenar o índice do elemento escolhido;
    vector<int> aux; // Vetor para auxiliar a escolha do alfa;

    // "i" representa o índice de cada alfa do vetor de alfas;
    for (int i = 0; i < prob.size(); i++)
    {
        // Cada "i" será adicionado no vetor "aux" uma quantidade de vezes equivalente à sua probabilidade multiplicado por 100;
        for (int j = 0; j < (int)(prob[i] * 100); j++)
        {
            aux.push_back(i);
        }
        // O vetor "aux" possuirá um total de 100 elementos disponíveis para sorteio;
    }

    // Sorteia aleatoriamente o índice do alfa no vetor "aux";
    auxIndex = rand() % aux.size();
    index = aux[auxIndex];

    return index;
}

//! Temporizador ****************************************************************

/**
 * @brief Inicia a contagem de tempo
 * @param inicio 
 */
void Graph::iniciaProces(chrono::time_point<chrono::system_clock> *inicio)
{
    *inicio = chrono::system_clock::now();
}

/**
 * @brief finaliza a contagem de tempo
 * @param inicio 
 * @param fim 
 * @return double 
 */
double Graph::fimProces(chrono::time_point<chrono::system_clock> *inicio, chrono::time_point<chrono::system_clock> *fim)
{
    *fim = chrono::system_clock::now();
    chrono::duration<double> tsegundos = *fim - *inicio;
    double temp = tsegundos.count();
    return temp;
}
//! *********************************************************************************