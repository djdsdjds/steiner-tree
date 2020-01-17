#include "grafos_lemon.h"
#include <chrono>
#include <unordered_set>
using namespace std;

int main()
{
  try {

    UGraph g("C:\\Users\\LENOVO\\Desktop\\grafos_lemon_entregar\\i640-132.txt");

    vector<int>t=g.getTerminals();
    cout<<"Nodos Terminais: ";
    for(size_t i=0;i<t.size();i++){
        cout<<t[i]<<' ';
    }
    cout<<"\n";

    cout<<"\nHeuristica Takahashi e Matsuyama\n";
    auto t3 = std::chrono::high_resolution_clock::now();
    heurout HTM=g.heur(t);
    auto t4 = std::chrono::high_resolution_clock::now();
    auto durationh = std::chrono::duration_cast<std::chrono::microseconds>( t4 - t3 ).count();
    cout<< "duracão heuristica:"<<durationh<< endl;
    for(set<Arc>::iterator it=HTM.graph.begin();it!=HTM.graph.end();it++){
        cout<<g.getOrigem(*it)<<' '<<g.getDestino(*it)<<'\n';
    }
    cout<<"custo: "<<HTM.cost<<"\n";
    cout << '\n';


    cout << "Arvore Steiner:\n";
    auto t1 = std::chrono::high_resolution_clock::now();
    dadosConj Steiner=g.Dreyfus_Wagner();
    auto t2 = std::chrono::high_resolution_clock::now();
    cout<<endl;
    cout<< "\ncusto: "<<Steiner.cost <<"\n";
    auto durationdw = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    cout<<"duracão dw:"<< durationdw << endl;
    set<pair<int,int>> st;
        for(size_t i=0;i<Steiner.idxP.size();i++){
            path_s p=g.dijkstra(Steiner.idxP[i].first,Steiner.idxP[i].second);
            vector<pair<int,int>>v=pares_arestas(p.path);
            for(size_t j=0;j<v.size();j++){
                int p1=v[j].first,p2=v[j].second;
                if(p2!=0){
                    if(p1>p2){swap(p1,p2);}
                    st.insert(make_pair(p1,p2));
                    }
                }
            }

        for(set<pair<int,int>>::iterator it=st.begin();it!=st.end();it++){
            cout<<(*it).first<<" "<<(*it).second<<endl;
        }
        cout<<endl;
        cout<< "\ncusto: "<<Steiner.cost <<"\n";
        cout<<"\n";



    return 0;
  }catch(const errGraph& e) {
    cerr << e.what() << '\n';
    return 1;
  }catch(const exception& e) {
    cerr << e.what() << '\n';
    return 2;
  }catch(...) {
    cerr << "excepção desconhecida\n";
    return 3;
  }

}
