#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <utility>
#include <tuple>
#include <iomanip>
#include <stdlib.h>
#include <chrono>
#include "Graph.h"
#include "Node.h"

using namespace std;

Graph *leitura(ifstream &input_file, int directed, int weightedEdge, int weightedNode)
{

    //Variáveis para auxiliar na criação dos nós no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;

    //Pegando a ordem do grafo
    input_file >> order;

    //Criando objeto grafo: Ordem - direcionado - peso Aresta - peso Nó
    Graph *graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo

    if (!graph->getWeightedEdge() && !graph->getWeightedNode())
    {

        while (input_file >> idNodeSource >> idNodeTarget)
        {

            graph->insertEdge(idNodeSource, idNodeTarget, 0);
        }
    }
    else if (graph->getWeightedEdge() && !graph->getWeightedNode())
    {

        float edgeWeight;

        while (input_file >> idNodeSource >> idNodeTarget >> edgeWeight)
        {
            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);
        }
    }
    else if (graph->getWeightedNode() && !graph->getWeightedEdge())
    {

        float nodeSourceWeight, nodeTargetWeight;

        while (input_file >> idNodeSource >> nodeSourceWeight >> idNodeTarget >> nodeTargetWeight)
        {
            graph->insertEdge(idNodeSource, idNodeTarget, 0);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);
        }
    }
    else if (graph->getWeightedNode() && graph->getWeightedEdge())
    {

        float nodeSourceWeight, nodeTargetWeight, edgeWeight;

        while (input_file >> idNodeSource >> nodeSourceWeight >> idNodeTarget >> nodeTargetWeight)
        {
            graph->insertEdge(idNodeSource, idNodeTarget, edgeWeight);
            graph->getNode(idNodeSource)->setWeight(nodeSourceWeight);
            graph->getNode(idNodeTarget)->setWeight(nodeTargetWeight);
        }
    }

    return graph;
}

// Ler arquivo grafo não direcionado e sem pesos
Graph *leituraInstancia(ifstream &input_file, int directed, int weightedEdge, int weightedNode)
{
    //Variáveis para auxiliar na criação dos nós no Grafo
    int idNodeSource;
    int idNodeTarget;
    int order;
    int numEdges;

    //Pegando a ordem do grafo
    input_file >> order >> numEdges;

    cout << "\nLendo o arquivo input.txt..." << endl;
    cout << "Ordem: " << order << " - Arestas: " << numEdges << endl;

    //Criando objeto grafo
    Graph *graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo
    while (input_file >> idNodeSource >> idNodeTarget)
    {
        cout << "Edge: " << idNodeSource << " - " << idNodeTarget << endl;
        graph->insertEdge(idNodeSource, idNodeTarget, 0);
    }

    return graph;
}

int menu()
{
    int selecao;
    cout << "\nMENU" << endl;
    cout << "----" << endl;
    cout << "[1] Fecho Transitivo Direto" << endl;
    cout << "[2] Fecho Transitivo Indireto" << endl;
    cout << "[3] Caminho Mínimo entre dois vértices - Dijkstra" << endl;
    cout << "[4] Caminho Mínimo entre dois vértices - Floyd" << endl;
    cout << "[5] Árvore Geradora Mínima sobre subgrafo vertice induzido por X usando algorítmo de Prim" << endl;
    cout << "[6] Árvore Geradora Mínima sobre subgrafo vertice induzido por X usando algorítmo de Kruskal" << endl;
    cout << "[7] Caminhamento Profundidade destacando as Arestas de retorno" << endl;
    cout << "[8] Ordenação topologica em D ou a informação de que não é um grafo acíclico direcionado" << endl;
    cout << "[9] Modulo de teste" << endl;
    cout << "[10] Salvar grafo em arquivo .dot - Não direcionado" << endl;
    cout << "[11] Salvar grafo em arquivo .dot - Direcionado" << endl;
    cout << "[0] Sair" << endl;

    cin >> selecao;

    return selecao;
}

void selecionar(int selecao, Graph *graph, ofstream &output_file)
{

    switch (selecao)
    {

    //Subgrafo induzido por um conjunto de vértices X;
    case 1:
    {
        break;
    }
        //Caminho mínimo entre dois vértices usando Dijkstra;
    case 2:
    {
        break;
    }

        //Caminho mínimo entre dois vértices usando Floyd;
    case 3:
    {
        break;
    }

        //AGM - Kruscal;
    case 4:
    {
        break;
    }

        //AGM Prim;
    case 5:
    {
        break;
    }
        //Busca em largura;
    case 6:
    {
        break;
    }
        //Ordenação Topologica;
    case 7:
    {
        break;
    }
    case 8:
    {
        break;
    }
    case 9:
    {
        graph->printGraph2();
        break;
    }
    case 10:
    {

        graph->printGraph_Dot_Not_Directed();
        break;
    }
        case 11:
    {

       
        break;
    }
    case 0:
    {
        exit(0);
        break;
    }

    default:
    {
        system("clear");
        cout << " Erro!!! Opção invalida." << endl;
    }
    }
    menu();
}

int mainMenu(ofstream &output_file, Graph *graph)
{

    int selecao = 1;

    while (selecao != 0)
    {
        // system("clear");
        selecao = menu();

        if (output_file.is_open())
            selecionar(selecao, graph, output_file);

        else
            cout << "Unable to open the output_file" << endl;

        output_file << endl;
    }

    return 0;
}

int main(int argc, char const *argv[])
{
    //Verificação se todos os parâmetros do programa foram entrados
    if (argc != 6)
    {
        cout << "ERRO: Esperado: ./<program_name> <input_file> <output_file> <directed> <weighted_edge> <weighted_node> " << endl;
        return 1;
    }

    string program_name(argv[0]);
    string input_file_name(argv[1]);

    string instance;
    if (input_file_name.find("v") <= input_file_name.size())
    {
        string instance = input_file_name.substr(input_file_name.find("v"));
        cout << "Running " << program_name << " with instance " << instance << " ... " << endl;
    }

    //Abrindo arquivo de entrada
    ifstream input_file;
    ofstream output_file;
    input_file.open(argv[1], ios::in);
    output_file.open(argv[2], ios::out | ios::trunc);

    Graph *graph;

    if (input_file.is_open())
    {

        graph = leituraInstancia(input_file, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
        //graph = leitura(input_file, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
    }
    else
        cout << "Não foi possível abrir o arquivo! " << argv[1];



    string d = "Sim";
    string a = "Sim";
    string v = "Sim";
    int o = graph->getDirected();
    int e = graph->getWeightedEdge();
    int n = graph->getWeightedNode();

    if (!o)
        d = "Não";
    if (!e)
        a = "Não";
    if (!n)
        v = "Não";

    cout << "Criando grafo..." << endl;
    cout << "Ordem: " << graph->getOrder() << endl;
    cout << "Nº Arestas: " << graph->getNumberEdges() << endl;
    cout << "Direcionado? " << d << endl;
    cout << "Arestas com peso? " << a << endl;
    cout << "Vertices com peso? " << v << endl;

    mainMenu(output_file, graph);

    //Fechando arquivo de entrada
    input_file.close();

    //Fechando arquivo de saída
    output_file.close();

    return 0;
}
