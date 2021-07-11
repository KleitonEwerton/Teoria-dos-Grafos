# Trabalho de Grafos
## Parte 1:

- [ ] Fecho Transitivo Direto
- [ ] Fecho Transitivo Indireto
- [ ] Caminho Mínimo entre dois vértices - Dijkstra
- [ ] Caminho Mínimo entre dois vértices - Floyd
- [ ] Árvore Geradora Mínima sobre subgrafo vertice induzido por X usando algorítmo de Prim
- [ ] Árvore Geradora Mínima sobre subgrafo vertice induzido por X usando algorítmo de Kruskal
- [x] Caminhamento Profundidade destacando as Arestas de retorno
- [ ] Ordenação topologica em D ou a informação de que não é um grafo acíclico direcionado
- [x] Modulo de teste
- [x] Salvar grafo em arquivo .dot - Não direcionado
- [x] Salvar grafo em arquivo .dot - Direcionado
## Parte 2:




## Como Compilar?
Cria o arquivo auto.exe e todos os arquivos .o

```
Make all
```

Exclui todos os arquivos .o e o arquivo ./auto.exe

```
Make clean
```

## Gerar o arquivo dot

Comando para criação do gráfico via o programa Grafix
...
dot -Tpng output.dot -o output.png
...
## Como rodar?

Grafo não direcionado, sem peso nas aresta e sem peso nos vértices 
...
./grafo.exe input.txt output.txt 0 0 0
...

Grafo direcionado 
...
./grafo.exe input.txt output.txt 1 0 0
...

Grafo com peso na aresta 
...
./grafo.exe input.txt output.txt 0 1 0
...

Grafo com peso no vértices 
...
./grafo.exe input.txt output.txt 0 0 1
...