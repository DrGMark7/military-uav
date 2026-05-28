// =====================================================================
//   Mock data is read from ./data/{nodes,tau,tauprime,Cprime}_01001.csv
// ===================================================================== 

#include <ilcplex/ilocplex.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <set>
#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>

ILOSTLBEGIN

// --------------------------------------------------------------------- 
//  DATA LAYER                                                            
// --------------------------------------------------------------------- 
struct ProblemData {
    int    C{};                // customer count                          
    int    N{};                // total nodes (incl. both depots)         
    int    B{};                // number of drones                        
    int    S{};                // sorties                                 
    double en{};               // drone endurance (min)                   
    double L{};                // launch / retrieval fixed time           
    double M1{};               // big-M                                   

    vector<vector<double>> tau, taup;
    vector<array<double,4>> nodes;

    vector<int> N_all, N0, Nplus, CC, Cprime, Ctruck;
    set<int>    Cprime_set;

    //  Load dataset from csv
    void loadCSV(const string& tag) {
        auto readMat = [](const string& p, int n) {
            vector<vector<double>> m(n, vector<double>(n,0.0));
            ifstream f(p); if (!f) throw runtime_error("Cannot open "+p);
            for (int i=0;i<n;++i) {
                string ln; getline(f,ln); istringstream ss(ln);
                for (int j=0;j<n;++j) { string t; getline(ss,t,','); m[i][j]=stod(t); }
            }
            return m;
        };
        { ifstream f("./data/nodes_"+tag+".csv");
          if (!f) throw runtime_error("Cannot open nodes_"+tag+".csv");
          string ln;
          while (getline(f,ln)) {
              istringstream ss(ln); array<double,4> r{};
              for (int k=0;k<4;++k) { string t; getline(ss,t,','); r[k]=stod(t); }
              nodes.push_back(r);
          }
        }
        N = (int)nodes.size();   C = N - 2;
        tau  = readMat("./data/tau_"      +tag+".csv", N);
        taup = readMat("./data/tauprime_" +tag+".csv", N);
        { ifstream f("./data/Cprime_"+tag+".csv");
          if (!f) throw runtime_error("Cannot open Cprime_"+tag+".csv");
          string ln; getline(f,ln); istringstream ss(ln); string t;
          while (getline(ss,t,',')) Cprime.push_back(stoi(t));
        }
        for (int v : Cprime) Cprime_set.insert(v);

        for (int i=0;i<N;  ++i) N_all.push_back(i);
        for (int i=0;i<N-1;++i) N0.push_back(i);
        for (int i=1;i<N;  ++i) Nplus.push_back(i);
        for (int i=1;i<=C; ++i) CC.push_back(i);
        for (int i=1;i<=C; ++i) if (!Cprime_set.count(i)) Ctruck.push_back(i);

        M1 = 1e4;
    }

    void print() const {
        cout << "  N=" << N << "  C=" << C
             << "  B=" << B << "  S=" << S
             << "  en=" << en << "  L=" << L << "\n";
        cout << "  C' (drone-only) = {"; for (int v:Cprime) cout<<v<<" "; cout<<"}\n";
        cout << "  C\\C' (truck)    = {"; for (int v:Ctruck) cout<<v<<" "; cout<<"}\n";
    }
};

// --------------------------------------------------------------------- 
//  MAIN                                                                  
// --------------------------------------------------------------------- 
int main() {
    cout << "+-----------------------------------------------------------+\n";
    cout << "|              FLOW constraints (F1 - F15) only             |\n";
    cout << "+-----------------------------------------------------------+\n";

    ProblemData d;
    d.B = 2;  d.S = 2;  d.en = 30.0;  d.L = 1.0;
    try { d.loadCSV("01001"); }
    catch (const exception& e) { cerr<<"CSV: "<<e.what()<<"\n"; return 1; }
    d.print();

    IloEnv env;
    try {
        IloModel model(env);
        const int N = d.N, B = d.B, S = d.S;

        // ------------- DECISION VARIABLES -------------
        //  x[i][j] : truck arc
        IloArray<IloBoolVarArray> x(env, N);
        for (int i=0;i<N;++i) {
            x[i] = IloBoolVarArray(env, N);
            for (int j=0;j<N;++j) {
                ostringstream nm; nm<<"x_"<<i<<"_"<<j;
                x[i][j] = IloBoolVar(env, nm.str().c_str());
            }
        }
        
        // y[b][s][i][j] : drone b traverses arc (i,j) in sortie s
        IloArray<IloArray<IloArray<IloBoolVarArray>>> y(env, B);
        for (int b=0;b<B;++b) {
            y[b] = IloArray<IloArray<IloBoolVarArray>>(env, S);
            for (int s=0;s<S;++s) {
                y[b][s] = IloArray<IloBoolVarArray>(env, N);
                for (int i=0;i<N;++i) {
                    y[b][s][i] = IloBoolVarArray(env, N);
                    for (int j=0;j<N;++j) {
                        ostringstream nm; nm<<"y_"<<b<<"_"<<s<<"_"<<i<<"_"<<j;
                        y[b][s][i][j] = IloBoolVar(env, nm.str().c_str());
                    }
                }
            }
        }

        // z[b][s][i][k] : drone b sortie s launches at i, rendezvous at k
        IloArray<IloArray<IloArray<IloBoolVarArray>>> z(env, B);
        for (int b=0;b<B;++b) {
            z[b] = IloArray<IloArray<IloBoolVarArray>>(env, S);
            for (int s=0;s<S;++s) {
                z[b][s] = IloArray<IloBoolVarArray>(env, N);
                for (int i=0;i<N;++i) {
                    z[b][s][i] = IloBoolVarArray(env, N);
                    for (int k=0;k<N;++k) {
                        ostringstream nm; nm<<"z_"<<b<<"_"<<s<<"_"<<i<<"_"<<k;
                        z[b][s][i][k] = IloBoolVar(env, nm.str().c_str());
                    }
                }
            }
        }
        
        // ------------- FLOW CONSTRAINTS F1 - F15 -------------

        // F1 : (truck departs starting depot)    
        {
            IloExpr e1(env);
            for (int j : d.CC) e1 += x[0][j];
            model.add(e1 == 1).setName("F1");
            e1.end();
        }

        // F2 : (truck enters ending depot from a truck-capable node)                                
        {
            IloExpr e2(env);
            for (int j : d.Ctruck) e2 += x[j][N-1];
            model.add(e2 == 1).setName("F2");
            e2.end();
        }

        // F3 : in-flow = out-flow at every truck-capable customer      
        for (int j : d.Ctruck) {
            IloExpr in_(env), out_(env);
            for (int i : d.N0)    if (i!=j) in_  += x[i][j];
            for (int k : d.Nplus) if (k!=j) out_ += x[j][k];
            model.add(in_ == out_).setName(("F3_"+to_string(j)).c_str());
            in_.end(); out_.end();
        }

        // F4 : drone flow balance         
        for (int j : d.CC) {
            for (int b=0;b<B;++b) {
                IloExpr lhs(env), rhs(env);
                for (int s=0;s<S;++s) {
                    for (int i : d.N0)    if (i!=j) lhs += y[b][s][i][j];
                    for (int l : d.N0)    if (l!=j) lhs -= z[b][s][l][j];
                    for (int k : d.Nplus) if (k!=j) rhs += y[b][s][j][k];
                    for (int m : d.Nplus) if (m!=j) rhs -= z[b][s][j][m];
                }
                model.add(lhs <= rhs).setName(("F4_"+to_string(b)+"_"+to_string(j)).c_str());
                lhs.end(); rhs.end();
            }
        }

        // F5 : rendezvous-at-k requires drone-arrival-at-k             
        for (int k : d.Nplus) for (int b=0;b<B;++b) for (int s=0;s<S;++s) {
            IloExpr lhs(env), rhs(env);
            for (int i : d.N0) if (i!=k) lhs += z[b][s][i][k];
            for (int j : d.CC) if (j!=k) rhs += y[b][s][j][k];
            model.add(lhs <= rhs).setName("F5");
            lhs.end(); rhs.end();
        }

        // F6 : launch-from-i requires drone-departure-from-i           
        for (int i : d.N0) for (int b=0;b<B;++b) for (int s=0;s<S;++s) {
            IloExpr lhs(env), rhs(env);
            for (int k : d.Nplus) if (k!=i) lhs += z[b][s][i][k];
            for (int j : d.CC)    if (j!=i) rhs += y[b][s][i][j];
            model.add(lhs <= rhs).setName("F6");
            lhs.end(); rhs.end();
        }

        // F7 : truck must visit the launch node                        
        for (int i : d.CC) for (int b=0;b<B;++b) {
            IloExpr lhs(env), rhs(env);
            for (int s=0;s<S;++s)
                for (int k : d.Nplus) if (k!=i) lhs += z[b][s][i][k];
            for (int l : d.N0) if (l!=i) rhs += x[l][i];
            model.add(lhs <= rhs).setName("F7");
            lhs.end(); rhs.end();
        }

        // F8 : if both truck-arc and drone-arc emanate from i, must launch 
        for (int i : d.N0) for (int b=0;b<B;++b) for (int s=0;s<S;++s) {
            IloExpr a(env), c(env);
            for (int j : d.Nplus) if (j!=i) a += x[i][j];
            for (int l : d.Nplus) if (l!=i) a += y[b][s][i][l];
            for (int k : d.Nplus) if (k!=i) c += z[b][s][i][k];
            model.add(a - 1 <= c).setName("F8");
            a.end(); c.end();
        }

        // F9 : drones return only to truck-capable nodes
        for (int k : d.Nplus) {
            if (d.Cprime_set.count(k)) continue;
            for (int b=0;b<B;++b) {
                IloExpr lhs(env), rhs(env);
                for (int s=0;s<S;++s) {
                    for (int j : d.N0) if (j!=k) lhs += y[b][s][j][k];
                    for (int i : d.N0) if (i!=k) rhs += z[b][s][i][k];
                }
                model.add(lhs == rhs).setName("F9");
                lhs.end(); rhs.end();
            }
        }

        // F10 : if z_{i,k} = 1 then truck arrives at i AND at k         
        for (int i : d.CC) for (int k : d.Nplus) {
            if (k==i) continue;
            for (int b=0;b<B;++b) for (int s=0;s<S;++s) {
                IloExpr rhs(env);
                for (int h : d.N0) if (h!=i && h!=k) rhs += x[h][i];
                for (int l : d.CC) if (l!=k)         rhs += x[l][k];
                model.add(2*z[b][s][i][k] <= rhs).setName("F10");
                rhs.end();
            }
        }

        // F11 : if z_{0,k} = 1 then truck enters k from some predecessor 
        for (int k : d.Nplus) for (int b=0;b<B;++b) for (int s=0;s<S;++s) {
            IloExpr rhs(env);
            for (int h : d.N0) if (h!=k) rhs += x[h][k];
            model.add(z[b][s][0][k] <= rhs).setName("F11");
            rhs.end();
        }

        // F12 : at most one launch per (b,s)                            
        for (int b=0;b<B;++b) for (int s=0;s<S;++s) {
            IloExpr e(env);
            for (int i : d.N0) for (int k : d.Nplus) if (k!=i) e += z[b][s][i][k];
            model.add(e <= 1).setName("F12");
            e.end();
        }

        // F13 : at most one rendezvous-at-k per (b,s)                    
        for (int k : d.Nplus) for (int b=0;b<B;++b) for (int s=0;s<S;++s) {
            IloExpr e(env);
            for (int i : d.N0) if (i!=k) e += z[b][s][i][k];
            model.add(e <= 1).setName("F13");
            e.end();
        }

        // F14 : prohibit drone flying directly launch -> rendezvous  (launch and rendezvous must be separated by at least one node)   
        for (int i : d.N0) for (int k : d.Nplus) {
            if (k==i) continue;
            for (int b=0;b<B;++b) for (int s=0;s<S;++s)
                model.add(y[b][s][i][k] + z[b][s][i][k] <= 1).setName("F14");
        }

        // F15 : each drone-only node visited at most once per sortie    
        for (int i : d.Cprime) for (int s=0;s<S;++s) {
            IloExpr e(env);
            for (int b=0;b<B;++b)
                for (int k : d.N_all) if (k!=i) e += y[b][s][k][i];
            model.add(e <= 1).setName("F15");
            e.end();
        }

        // Miscellaneous constraints 
        // M1 : x_ii = 0                                                    
        for (int i=0;i<N;++i) model.add(x[i][i] == 0);

        // write objective function here
    }
    env.end();
    return 0;
}
