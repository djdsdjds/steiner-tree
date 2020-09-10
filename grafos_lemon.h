//**********************************************
// not my code, header given by the professor
//**********************************************











#ifndef GRAPH_LEMON_H
#define GRAPH_LEMON_H



#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <algorithm>
#include <cmath>
#include <limits>


#include <lemon/core.h>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <lemon/kruskal.h>
#include <lemon/path.h>
#include <lemon/bfs.h>
#include <lemon/dfs.h>
#include <lemon/connectivity.h>
#include <lemon/preflow.h>
#include <lemon/capacity_scaling.h>
#include <lemon/concepts/digraph.h>

typedef lemon::ListGraph Graph;
typedef Graph::EdgeMap<double> EdgeMapDouble;
typedef Graph::EdgeMap<int> EdgeMapInt;
typedef Graph::NodeMap<int> NodeMapInt;
typedef Graph::Node Node;
typedef Graph::Edge Edge;
typedef Graph::Arc Arc;

//Estrutura para armazenar caminhos

struct path_s {
    double cost;
    std::list<int> path; //armazena os ids dos vértices
};

//Estrutura para armazenar arestas e a soma dos custos delas
//Usado nos algoritmos MST

struct edges_s {
    double cost;
    std::vector<std::pair<int,int>> edges;
};

struct dadosConj {
    std::set<int> nodos; //nodos de X
    std::vector< std::pair<int,int> > idxP; //pares (u,v) para obter os caminhos em P_{uv}
    double cost;
};
struct heurout{
    std::set<Arc> graph;
    double cost;
};
class errGraph
{
  private:
    std::string msg;
  public:
    explicit errGraph(const std::string& s):msg(s) {}
    const std::string& what() const {return msg;}
};

class UGraph
{
  private:
    Graph g;                     //grafo
    EdgeMapDouble *costs;         //custo de cada aresta. Tem que ser inic no constr
    NodeMapInt   *meuId;         //Mapeamento entre os nodos e os nossos ids
    std::map<int,Node> id2node;  //Mapeamento entre os nossos ids e os nodos
    std::vector<int>terms;
  public:
    int getOrigem(const Arc& a){return getNodeId(g.source(a));}
    int getDestino(const Arc& a){return getNodeId(g.target(a));}
    //Default constructor
    UGraph() { initMaps(); }

    //Construtor com o número de nodos
    UGraph(size_t n) {
      initMaps();
      g.reserveNode(n);
      for(size_t i=0;i<n;++i) {
    Node u=g.addNode();
    id2node[i+1]=u;
    (*meuId)[u]=i+1;
      }
    }

    //Construtor a partir de um ficheiro
    UGraph(const std::string& fich) {
      initMaps();
      readUndirectedGraphFromFile(fich);
    }

    //Destrutor
    ~UGraph() {
      delete costs;
      delete meuId;
    }

    //Copy constructor
    UGraph(const UGraph& s) {
      initMaps();
      //cópia do grafo
      lemon::GraphCopy<Graph, Graph> cg(s.g, g); //de origem para destino
      //cópia do mapeamento das arestas
      cg.edgeMap(*s.costs, *costs);
      //cópia do mapeamento dos nodos
      cg.nodeMap(*s.meuId, *meuId);
      cg.run();
      //criamos id2node
      for(Graph::NodeIt it(g);it!=lemon::INVALID;++it) {
    id2node[(*meuId)[it]]=it;
      }
      terms=s.terms;
    }

    //Operador =
    UGraph& operator=(const UGraph& s) {
      g.clear();
      //cópia do grafo
      lemon::GraphCopy<Graph, Graph> cg(s.g, g); //de origem para destino
      //cópia do mapeamento das arestas
      cg.edgeMap(*s.costs, *costs);
      //cópia do mapeamento dos nodos
      cg.nodeMap(*s.meuId, *meuId);
      cg.run();
      //recriamos id2node
      id2node.clear();
      for(Graph::NodeIt it(g);it!=lemon::INVALID;++it) {
    id2node[(*meuId)[it]]=it;
      }
      terms=s.terms;
      return *this;
    }

    int numNodes() const {return lemon::countNodes(g);}
    int numEdges() const {return lemon::countEdges(g);}

    void addEdge(int id_u,int id_v, double w) {
      if(id2node.find(id_u)==id2node.end())
    throw errGraph{"addEdge: node "+std::to_string(id_u)+" does not exist"};
      if(id2node.find(id_v)==id2node.end())
    throw errGraph{"addEdge: node "+std::to_string(id_v)+" does not exist"};
      if(edgeExists(id_u,id_v))
    throw errGraph{"addEdge: edge ("+std::to_string(id_u)+","+
        std::to_string(id_v)+") already exists"};
      Edge e=g.addEdge(id2node.at(id_u),id2node.at(id_v));
      (*costs)[e]=w;
    }


    bool edgeExists(int id_u, int id_v) const {
      Node u=id2node.at(id_u);
      Node v=id2node.at(id_v);

      for(Graph::OutArcIt it(g,u);it!=lemon::INVALID;++it) {
    if(g.target(it)==v) return true;
      }
      return false;
    }

    void eraseEdge(int id_u, int id_v) {
      if(id2node.find(id_u)==id2node.end())
    throw errGraph{"eraseEdge: node "+std::to_string(id_u)+" does not exist"};
      if(id2node.find(id_v)==id2node.end())
    throw errGraph{"eraseEdge: node "+std::to_string(id_v)+" does not exist"};

      Node u=id2node.at(id_u);
      Node v=id2node.at(id_v);
      for(Graph::OutArcIt it(g,u);it!=lemon::INVALID;++it) {
    if(g.target(it)==v) {g.erase(it); return;}
      }

      throw errGraph{"eraseEdge: edge ("+std::to_string(id_u)+","+
      std::to_string(id_v)+") dows not exist"};
    }

    void addNode(int id_u) {
      if(id2node.find(id_u)!=id2node.end())
    throw errGraph{"addNode: node "+std::to_string(id_u)+" already exists"};
      Node u=g.addNode();
      id2node[id_u]=u;
      (*meuId)[u]=id_u;
    }

    void eraseNode(int id_u) {
      if(id2node.find(id_u)==id2node.end())
    throw errGraph{"eraseNode: node "+std::to_string(id_u)+" does not exist"};

      Node u=id2node.at(id_u);
      g.erase(u);
      id2node.erase(id_u);
    }

    int getNodeId(Node u) const {return (*meuId)[u];}

    //**********************************************
    // Dijkstra todos os caminhos a partir da origem
    //**********************************************


    //Dijkstra para encontrar todos os caminhos a partir de uma origem

    std::vector<path_s> dijkstra(int id_source) const {

      //a origem tem que existir
      if(id2node.find(id_source)==id2node.end())
    throw errGraph{"dijkstra: invalid source: "+std::to_string(id_source)};

      //Não pode haver custos negativos

      for(Graph::EdgeIt it(g);it!=lemon::INVALID;++it)
    if((*costs)[it]<0)
      throw errGraph{"dijkstra: custo negativo."};

      Node source=id2node.at(id_source);

      std::vector<path_s> ret;


      lemon::Dijkstra<Graph,EdgeMapDouble> djks(g,*costs);
      djks.run(source);

      for (Graph::NodeIt it(g); it != lemon::INVALID; ++it) {
    if (djks.reached(it)) {
      if(it==source) continue;
      path_s p;

      lemon::Path<Graph> pp=djks.path(it);
      for(int i=0;i<pp.length();++i)  {
        p.path.push_back(getNodeId(g.source(pp.nth(i))));
      }
      p.path.push_back(getNodeId(g.target(pp.back())));


      //p.path=buildListFromPred(djks.predMap(),source,it);
      p.cost=djks.dist(id2node.at(p.path.back()));
      ret.push_back(p);
    }
      }

      return ret;
    }

    //**********************************************
    // Dijkstra caminho origem -> dest
    //**********************************************

    //Dijkstra para encontrar o caminho entre origem e destino
    path_s dijkstra(int id_source, int id_target)  const {

      if(id2node.find(id_source)==id2node.end())
    throw errGraph{"dijkstra: invalid source: "+std::to_string(id_source)};
      if(id2node.find(id_target)==id2node.end())
    throw errGraph{"dijkstra: invalid target: "+std::to_string(id_target)};

      //Não pode haver custos negativos

      for(Graph::EdgeIt it(g);it!=lemon::INVALID;++it)
    if((*costs)[it]<0)
      throw errGraph{"dijkstra: custo negativo."};

      Node source=id2node.at(id_source);
      Node target=id2node.at(id_target);

      path_s ret; ret.cost=0;

      if(source==target) return ret;

      lemon::Dijkstra<Graph,EdgeMapDouble> djks(g,*costs);
      djks.run(source,target);


      if(!djks.reached(target)) return ret;

      lemon::Path<Graph> p=djks.path(target);
      for(int i=0;i<p.length();++i)  {
    ret.path.push_back(getNodeId(g.source(p.nth(i))));
      }
      ret.path.push_back(getNodeId(g.target(p.back())));

      //ret.path=buildListFromPred(djks.predMap(),source,target);
      ret.cost=djks.dist(target);

      return ret;
    }

    //**********************************************
    // Minimum Spanning Tree
    //     Se o grafo não for conexo devolve uma
    //     floresta
    //**********************************************

    edges_s kruskal() const {
      edges_s ret;

      std::vector<Edge> tree;
      ret.cost=lemon::kruskal(g,*costs,std::back_inserter(tree));

      for(size_t i=0;i<tree.size();++i) {
    std::pair<int,int> p(getNodeId(g.u(tree[i])),getNodeId(g.v(tree[i])));
    ret.edges.push_back(p);
      }
      return ret;
    }



    //**********************************************
    // Componentes conexas
    //**********************************************

    std::vector<std::vector<int>> connectedComponents() {
      NodeMapInt m(g);
      int n=lemon::connectedComponents(g,m);

      if(n==0) return std::vector<std::vector<int>>();
      std::vector<std::vector<int>> ret(n);

      for(Graph::NodeIt it(g);it!=lemon::INVALID;++it)
    ret[m[it]].push_back(getNodeId(it));
      return ret;
    }

    //**********************************************
    // Output do grafo
    //**********************************************

    void printGraph() {
      for(Graph::NodeIt it_u(g);it_u!=lemon::INVALID;++it_u) {
    if(!countOutArcs(g,it_u)) {
      std::cout << getNodeId(it_u) << " <-->\n";
      continue;
    }
    for(Graph::OutArcIt it_a(g,it_u); it_a!=lemon::INVALID;++it_a) {
      if(getNodeId(g.target(it_a)) <  getNodeId(g.source(it_a))) continue;
      std::cout << getNodeId(g.source(it_a)) << " <--> " <<
        getNodeId(g.target(it_a)) << " ("<<(*costs)[it_a]<<')';
      std::cout << '\n';
    }
      }
    }



    //**********************************************
    // Leitura do grafo a partir de um ficheiro
    //**********************************************


    void readUndirectedGraphFromFile(const std::string& fich)  {
        std::ifstream ifs(fich);
        if(!ifs) throw errGraph{"readUndirectedGraphFromFile: failed to open file "+fich};

        //lemos o cabeçalho
        //numero de nodos, numero de edgeos ou arestas, tipo de grafo (1 orientado, 2 não orientado),
        //info edgeos 1 só pesos, 2 pesos e capacidades

        int nNodes,nEdges,nTerminals;
        std::vector<std::string> v;
        if(!readLine(ifs,v))
      throw errGraph{"readUndirectedGraphFromFile: file does not contain any headers"};

        //Estamos a ler um grafo não orientado pelo que a primeira linha tem que conter
        //3 valores.


        if(v.size()!=3)
      throw errGraph{"readUndirectedGraphFromFile: header error. For undirected graphs you must provide 3 values"};

        nNodes=std::stoi(v[0]); nEdges=std::stoi(v[1]); nTerminals=std::stoi(v[2]);

        if(nNodes<=0 || nEdges<=0)
      throw errGraph{"readUndirectedGraphFromFile: number of nodes and number of edges must be >0"};


        g.reserveNode(nNodes);
        g.reserveEdge(nEdges);

        //Criamos os nodos todos. Os nodos estão numerados de 1 a nNodes
        for(int i=0;i<nNodes;++i) {
      Node u=g.addNode();
      id2node[i+1]=u;
      (*meuId)[u]=i+1;
        }

        if(!readLine(ifs,v))
      throw errGraph{"readUndirectedGraphFromFile: file does not contain any headers"};
        for(size_t i=0;i<nTerminals;i++){
            terms.push_back(std::stoi(v[i]));
        }
        int count=0;

        while(readLine(ifs,v)) {
      if(v.size()!=3)
        throw errGraph{"readUndirectedGraphFromFile: wrong number of values in adjacencies"};

      int s,t;
      double cost;
      s=std::stoi(v[0]); t=std::stoi(v[1]);
      if(s<1 || s>nNodes || t<1 || t>nNodes)
        throw errGraph{"readUndirectedGraphFromFile: invalid node"};
      cost=std::stod(v[2]);
      addEdge(s,t,cost);
      count ++;
        }

        ifs.close();
        if(count!=nEdges) throw errGraph{"readUndirectedGraphFromFile: number of edges not as specified in header"};
    }
    const std::vector<int>getTerminals() const{
        return terms;
    }
    heurout heur (std::vector<int> TB);
    std::list<Arc> buildListFromPred (const Graph::NodeMap<Arc>& pred,const Node& t);

    std::set<int>vetorparaset(std::vector<int>v);
    size_t indiceconjunto(std::vector<std::vector<dadosConj>>S, size_t nodo, std::vector<int>v);
    std::set<std::vector<int>> subsets (std::vector<int> nodos);
    std::vector<int>complementar(std::vector<int>pai, std::vector<int> filho);
    dadosConj Dreyfus_Wagner();
  private:

    void initMaps() {
      //chamado dos construtores
      costs=new EdgeMapDouble(g);
      meuId=new NodeMapInt(g);
    }

    bool readLine(std::ifstream& ifs, std::vector<std::string>& v) {
      //devolve true se conseguir ler alguma coisa, false caso contrário
      bool lido=false;
      std::string str;
      v.clear();
      do {
    std::getline(ifs,str);
    if(ifs.eof()) return false;
    size_t n=str.find_first_not_of(" \t");
    if(n==std::string::npos) continue; //linha vazia
    if(str[n]=='#') continue;
    lido=true;
      }while(!lido);

      //Partimos a string em campos separados por espaços ou tabs
      std::istringstream ss(str);
      while (!ss.eof()) {
    std::string x;
    ss >> x;
    if(x.length()) v.push_back(x);
      }

      return true;
    }

};
std::set<int>unir_sets(std::set<int>a,std::set<int>b);

std::vector<std::pair<int,int>>pares_arestas(std::list<int>l);
#endif
