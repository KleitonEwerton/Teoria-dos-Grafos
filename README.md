# Trabalho de Grafos

## Equipe

Matrícula       Nome                                E-mail
```
202065020A      Daniel Muller Rezende               daniel.rezende@estudante.ufjf.br
201876043       Deoclécio Porfírio Ferreira Filho   deoclecio.filho@estudante.ufjf.br
202065500B      Guilherme Martins Couto
202065050A      Kleiton Ewerton de Oliveira         kleitonewertonoliveira@gmail.com
```
## Parte 1: Funcionalidades básicas em grafos simples

- [X] Fecho Transitivo Direto
- [X] Fecho Transitivo Indireto
- [X] Caminho Mínimo entre dois vértices - Dijkstra
- [X] Caminho Mínimo entre dois vértices - Floyd
- [X] Árvore Geradora Mínima sobre subgrafo vertice induzido por X usando algorítmo de Prim
- [X] Árvore Geradora Mínima sobre subgrafo vertice induzido por X usando algorítmo de Kruskal
- [X] Caminhamento Profundidade destacando as Arestas de retorno
- [X] Ordenação topologica em D ou a informação de que não é um grafo acíclico direcionado
- [X] Modulo de teste
- [X] Salvar grafo em arquivo .dot - Não direcionado
- [X] Salvar grafo em arquivo .dot - Direcionado


## Parte 2: Apresentar os algoritmos abaixo para o problema:
### Arvore Geradora Mínima com Restrição de Grau

- [ ] Algoritmo guloso
- [ ] Algoritmo guloso randomizado;
- [ ] Algoritmo guloso randomizado reativo;


## Como Compilar?

Utilizando os comandos do Makefile:

Exclui todos os arquivos .o e o arquivo ./execGrupo22.exe
Cria o arquivo execGrupo22.exe e todos os arquivos .o

```
Make
```

Exclui todos os arquivos .o e o arquivo ./execGrupo22.exe

```
Make clean
```
## Como rodar?

Grafo não direcionado, sem peso nas aresta e sem peso nos vértices 
```
./execGrupo22.exe input.txt output.txt 0 0 0
```

Grafo direcionado 
```
./execGrupo22.exe input.txt output.txt 1 0 0
```

Grafo com peso na aresta 
```
./execGrupo22.exe input.txt output.txt 0 1 0
```

Grafo com peso no vértices 
```
./execGrupo22.exe input.txt output.txt 0 0 1
```
## Gerar o arquivo dot

Comando para instalação do programa Graphviz (linux)

```
sudo apt install graphviz
```
Comando para criação do gráfico via o programa Graphviz
```
dot -Tpng output.dot -o output.png
```
