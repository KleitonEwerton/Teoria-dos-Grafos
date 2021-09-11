#ifndef BEAD1A9D_8DDE_4C9D_8F7F_7E7E8D87BCE9
#define BEAD1A9D_8DDE_4C9D_8F7F_7E7E8D87BCE9
#include <vector>
using namespace std;

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


int randAlfa(float alfa, int tam_vetor);
int escolheAlfa(vector<float> &prob);

void atualizaMedias(vector<media> &medias, int solucao, vector<float> &alfas, float alfa);
void atualizaProbabilidades(vector<media> &medias, vector<float> &prob, vector<float> &solBest, vector<float> &q);

void inicializaVetores(vector<float> &prob, vector<media> &medias, int &numAlfas);

int pesoTotalAGM(vector<pair<int, int>> &pesoNoTotal);

void imprimirMatriz(vector<vector<pair<int, int>>> &matriz);
void imprimeSolucao(vector<pair<int, pair<int, int>>> &AGM_RESULTANTE, int posInicial, float alfa);

#endif /* BEAD1A9D_8DDE_4C9D_8F7F_7E7E8D87BCE9 */
