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
    int numEdges;
    float edgeWeight;

    //Pegando a ordem do grafo
    input_file >> order >> numEdges;

    cout << "\nLendo o arquivo input.txt..." << endl;
    cout << "Ordem: " << order << " - Arestas: " << numEdges << endl;

    //Criando objeto grafo: Ordem - direcionado - peso Aresta - peso Nó
    Graph *graph = new Graph(order, directed, weightedEdge, weightedNode);

    //Leitura de arquivo

    if (!graph->getWeightedEdge() && !graph->getWeightedNode())
    {

        while (input_file >> idNodeSource >> idNodeTarget)
        {
            cout << "Edge: " << idNodeSource << " - " << idNodeTarget << endl;
            graph->insertEdge(idNodeSource, idNodeTarget, 0);
        }
    }
    else if (graph->getWeightedEdge() && !graph->getWeightedNode())
    {
        while (input_file >> idNodeSource >> idNodeTarget >> edgeWeight)
        {
            cout << "Edge: " << idNodeSource << " - " << idNodeTarget << " - " << edgeWeight << endl;
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
        //Sair;
    case 0:
        exit(0);
        break;
    case 1: // Fecho Transitivo Direto;
        int v;
        do{
            cout << "Digite o id do nó: ";
            cin >> v;
        }while(!graph->searchNode(v));

        cout << "Fecho transitivo direto: ";
        graph->directTransitiveClosing(v);
        cout << endl;
        break;
        
    case 2: // Fecho Transitivo Indireto;
        break;
        
    case 3: //Caminho Mínimo entre dois vértices - Dijkstra

        int sourceD, targetD;
        try{   

        cout <<"Digite o node Source" << endl;
        cin >> sourceD;
        cout << "Digite o node Target"<< endl;
        cin >> targetD;
        cout << "A distância é: " << graph->dijkstra(sourceD,targetD)<<endl;
            
        }
        catch(const exception& e){ 

            cout << "Parâmetros inválidos" << endl;

        }

        break;
        
    case 4: // Caminho Mínimo entre dois vértices - Floyd
        break;
        
    case 5: //Árvore Geradora Mínima sobre subgrafo vertice induzido por X usando algorítmo de Prim
        break;
        
    case 6: //Árvore Geradora Mínima sobre subgrafo vertice induzido por X usando algorítmo de Kruskal
        break;
        
    case 7: //Caminhamento Profundidade destacando as Arestas de retorno
        int n;
        cout << "\n --- Caminhamento Profundidade destacando as Arestas de retorno --- \n\n";
        do
        {
            cout << "Informe o numero no nó: ";
            cin >> n;
        } while (!graph->searchNode(n));

        graph->deepSearch(n);
        break;

    case 8: //Ordenação topologica em D ou a informação de que não é um grafo acíclico direcionado
        break;

    case 9: //Modulo de teste
        graph->printGraph2();
        break;

    case 10: //Salvar grafo em arquivo .dot - Não direcionado
        graph->printGraph_Dot_Not_Directed();
        break;

    case 11: //Salvar grafo em arquivo .dot - Direcionado
        graph->printGraph_Dot_Directed();
        break;
        
    default:
        system("clear");
        cout << " Erro!!! Opção invalida." << endl;
    }
    menu();
}

int mainMenu(ofstream &output_file, Graph *graph)
{

    int selecao = -1;

    while (selecao != 0)
    {
        // system("clear");
        selecao = menu();

        if(!selecao)
        exit(0);
        
        if (output_file.is_open())
            selecionar(selecao, graph, output_file);
        else
            cout << "Arquivo não encontrado!" << endl;
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

        //graph = leituraInstancia(input_file, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
        graph = leitura(input_file, atoi(argv[3]), atoi(argv[4]), atoi(argv[5]));
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
