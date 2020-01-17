#include "grafos_lemon.h"

using namespace std;
set<int>unir_sets(set<int>a,set<int>b){
    set<int>uniao;
    for(set<int>::iterator it=a.begin();it!=a.end();it++){
        uniao.insert(*it);
    }
    for(set<int>::iterator it1=b.begin();it1!=b.end();it1++){
        uniao.insert(*it1);
    }
    return uniao;
}

//Algoritmo Dreyfus Wagner
vector<pair<int,int>>pares_arestas(list<int>l){
    list<int>m=l;
    vector<pair<int,int>>p;
    while(l.size()!=0){
        int u=*l.begin(); //1 2 3 4
        l.pop_front();
        int v=*l.begin();
        p.push_back(make_pair(u,v));
    }
    return p;
}
set<int>UGraph::vetorparaset(vector<int>v){
    set<int>s;
    for(size_t i=0;i<v.size();i++){
        s.insert(v[i]);
    }
    return s;
}
size_t UGraph::indiceconjunto(vector<vector<dadosConj>>S,size_t nodo,vector<int>v){
    size_t idx = 0;
    set<int>s=vetorparaset(v);
    for(size_t j=0;j<S[nodo].size();j++){
        if(S[nodo][j].nodos==s){
            idx=j;
            break;
        }

    }
    return idx;
}

vector<int>UGraph::complementar(vector<int>pai,vector<int>filho){
    vector<int> comp;
    for(int i=0;i<pai.size();i++){
        int aux=0;
        for(int j=0;j<filho.size();j++){
            if(filho[j]==pai[i]){
                aux=1;
            }
        }
        if(aux==0){
            comp.push_back(pai[i]);
        }
    }
    return comp;

}

set<vector<int>>UGraph::subsets (vector<int> nodos){
        set<vector<int>> list;
        for (int i = 0; i < pow(2,nodos.size()); i++)
        {
            vector<int> subset;

            for (int j = 0; j < nodos.size(); j++)
            {
                if ((i & (1 << j)) != 0)  {
                    subset.push_back(nodos[j]);
                }
            }
            list.insert(subset);
        }
        return list;
    }

dadosConj UGraph::Dreyfus_Wagner(){
    int n=numNodes();
    vector<map<set<int>,size_t>> indice;
    for(int i=0;i<n+1;i++){
        map<set<int>,size_t> aux;
        indice.push_back(aux);
    }
    vector<vector<dadosConj>> S(n+1);
    vector<map<vector<int>,size_t>> idx(n+1);
    vector<int>termos=getTerminals();
    int q=termos[0];
    termos.erase(termos.begin());//N-{q}
    vector<vector <path_s>> dijkstrac(n+1,vector<path_s>(n+1));
    for(int i=1;i<n+1;i++){
        for(int j=1;j<n+1;j++)
            if(i>=j){
                dijkstrac[i][j]=dijkstra(i,j);
            }
    } for(int i=1;i<n+1;i++){
        for(int j=1;j<n+1;j++)
            if(i>=j){
                dijkstrac[j][i]=dijkstrac[i][j];
            }
    }


    //passo 1
    for(size_t t=0;t<termos.size();t++) { //For each t in T-{q}
        for(size_t v=1;v<n+1;v++) {
            set<int> s;
            s.insert(termos[t]);
            dadosConj ct;
            ct.nodos=s;
            ct.cost=dijkstrac[v][termos[t]].cost;
            int p1=v, p2=termos[t];
            if(p1>p2){
                swap(p1,p2);
            }
            ct.idxP.push_back(make_pair(p1,p2));
            S[v].push_back(ct);
            indice[v][ct.nodos]=S[v].size()-1;

        }
    }

    //passo 2
    set<vector<int>>D=subsets(termos);
    for(int l=2;l<n-1;l++){
    for(set<vector<int>>::iterator it=D.begin();it!=D.end();it++){


            if((it->size()==l)){
                for(size_t v=1;v<n+1;v++) {

                    dadosConj ct;
                    ct.nodos=vetorparaset(*it);
                    ct.cost=numeric_limits<double>::infinity();
                    for(size_t i=0;i<(*it).size();i++){
                        int p1=v, p2=(*it)[i];
                        if(p1>p2){ swap(p1,p2);}

                        ct.idxP.push_back(make_pair(p1,p2));
                    }
                    S[v].push_back(ct);
                    indice[v][ct.nodos]=S[v].size()-1;
               }
               int z=(*it)[0];//escolher z arbitrário
               set<vector<int>>D_linea=subsets(*it);
               for(size_t w=1;w<n+1;w++){//para cada wEV
                   double s=numeric_limits<double>::infinity();
                   vector< pair<int,int> >path1,path2;
                   for(set<vector<int>>::iterator it2=D_linea.begin();it2!=D_linea.end();it2++){

                       //cada D' de D com z em D'
                        if(((*it2).size()>=1) && ((*it2).size()<(*it).size())){
                            vector<int>aux=*it2;
                            bool auxb=false;
                            for(int i=0;i<aux.size();i++){
                                if(aux[i]==z){
                                    auxb=true;
                                }
                            }


                            if(auxb){

                                vector<int>complement=complementar((*it),(*it2));
                                size_t idxD_linea = indice[w][vetorparaset(*it2)];
                                size_t idxD_linea_comp = indice[w][vetorparaset(complement)];
                                //guardar os idx correspondentes a D' e D\D'
                                path1=S[w][idxD_linea].idxP;
                                path2=S[w][idxD_linea_comp].idxP;

                                if(s>S[w][idxD_linea].cost+S[w][idxD_linea_comp].cost){
                                    s=S[w][idxD_linea].cost+S[w][idxD_linea_comp].cost;

                                }

                            }

                        }
                   }

                   //para cada vEV
                   for(size_t x=1;x<n+1;x++){

                       size_t idxD=indice[x][vetorparaset(*it)];
                       if(S[x][idxD].cost>dijkstrac[x][w].cost+s){
                            S[x][idxD].cost=dijkstrac[x][w].cost+s;
                            S[x][idxD].idxP.clear();
                            S[x][idxD].idxP.push_back(make_pair(x,w));
                            for(const auto& p:path1){ S[x][idxD].idxP.push_back(p);}
                            for(const auto& p:path2){ S[x][idxD].idxP.push_back(p);}
                   }

                 }

               }
            }
        }
    }

    dadosConj SP;

    //passo 3
    
    double C=numeric_limits<double>::infinity();
    int z=termos[0];
    int wSafe = 0;
    size_t idx1 = 0,idx2 = 0;
    for(size_t w=1;w<n+1;w++){
        set<vector<int>>D_linea=subsets(termos);
        double s=numeric_limits<double>::infinity();
        for(set<vector<int>>::iterator it=D_linea.begin();it!=D_linea.end();it++){
             if(((*it).size()>=1) && ((*it).size()<termos.size())){
                 vector<int>aux=(*it);
                 bool auxc=false;
                 for(int i=0;i<aux.size();i++){
                     if(aux[i]==z){
                         auxc=true;
                     }
                 }
                 if(auxc){
                     size_t idx_D_linea=indice[w][vetorparaset((*it))];
                     size_t idx_D_linea_comp=indice[w][vetorparaset(complementar(termos,(*it)))];
                    if(s>S[w][idx_D_linea].cost+S[w][idx_D_linea_comp].cost){
                        s=S[w][idx_D_linea].cost+S[w][idx_D_linea_comp].cost;
                        idx1=idx_D_linea;
                        idx2=idx_D_linea_comp;

                }
             }
            }
        }
        if((C>dijkstrac[q][w].cost+s)){
            C=dijkstrac[q][w].cost+s;
            wSafe=w;
        }
    }
    SP.cost=C;
    SP.idxP.push_back(make_pair(q,wSafe));
    for(size_t aux=0;aux<S[wSafe][idx1].idxP.size();aux++){
        SP.idxP.push_back(S[wSafe][idx1].idxP[aux]);
    }
    for(size_t aux1=0;aux1<S[wSafe][idx2].idxP.size();aux1++){
        SP.idxP.push_back(S[wSafe][idx2].idxP[aux1]);
    }
    SP.nodos=unir_sets(S[wSafe][idx1].nodos,S[wSafe][idx2].nodos);
    return SP;
}

//Heuristica Takahashi e Matsuyama

list<Arc> UGraph::buildListFromPred(const Graph::NodeMap<Arc>& pred,const Node& t) {
    //cria lista de ids a partir dos predecessores (os predecesores são arcos)
    std::list<Arc> ret;
    Arc prev = pred[t];
    while (prev != lemon::INVALID) {
        ret.push_front(prev);
        prev = pred[g.source(prev)];
    }
        return ret;
}

heurout UGraph::heur (std::vector<int> TB){
    vector<int> a=TB;
    vector<int> sk;
    set <Arc> steiner;
    int k=1;
    int asize=a.size();
    sk.push_back(a.back());
    a.pop_back();
    while(k<asize+1){
        list<Arc> caminho;
        double aux=numeric_limits<double>::infinity();
        for(size_t i=0;i<a.size();i++){
            for(size_t j=0;j<sk.size();j++){
                lemon::Dijkstra<Graph,EdgeMapDouble>djks(g,*costs);
                djks.run(id2node[sk[j]],id2node[a[i]]);
                list<Arc>l=buildListFromPred(djks.predMap(),id2node[a[i]]);
                double distancia=djks.dist(id2node[a[i]]);
                if(aux>distancia){
                    caminho=l;
                    aux=distancia;
                }
            }
        }

        for(const auto& it:caminho){
            steiner.insert(it);
            sk.push_back(getDestino(it));
            a.erase(remove(a.begin(), a.end(), getDestino(it)), a.end());

        }
        k=k+1;

    }
    heurout output;
    output.graph=steiner;
    double custo = 0.0;//aqui comparo edge e arc não será preferivel mudar para a list ser de Edges
    for(const auto& it:steiner){
        for(Graph::EdgeIt e(g); e!=lemon::INVALID; ++e){
            if((g.source(it)==g.u(e) && g.target(it)==g.v(e))||(g.source(it)==g.v(e) && g.target(it)==g.u(e))){
                custo=custo+(*costs)[e];
            }
        }
    }
    output.cost=custo;


    return output;

}
