#include <iostream>
#include <vector>
#include <typeinfo>
#include <queue>
#include <stack>
#include <set>
#include <iterator>
#include <tuple>


using namespace std;



class Graph{
public:
    vector<vector<int>>adjacencyList;   //ALSO MAY TRY MATRIX REPRESENTATION
    vector<int>indegrees; //may use
    vector<pair<int,pair<int,int>>>edges; //push the dges one by one in main!!! first=vertex second.first=destination second.second=distance
    int numVertices;
    Graph(int numVertices,bool weighted){
        this->numVertices=numVertices;
        vector<vector<int>>v(numVertices);
        this->adjacencyList=v;
        vector<int>v2(numVertices);
        indegrees=v2;

        for (int i = 0; i < numVertices; i++) {
            adjacencyList[i].resize((weighted)?numVertices:0);   //weighted ise indexlerde distance tutulur, değilse ajcacent node lar tutulur
        }


    }
    void dfs(int s);
    void dfsRecursive(int s,vector<bool>&visited);
    void bfs(int s,int aim,vector<int>&parents);
    void addEdge(int s,int u);
    void addEdge(int s,int u,int weight);
    void addEdgewithPair(int s,int u,int w);
    void addUndirectedEdge(int s,int u);
    void addUndirectedEdge(int s,int u,int weight);
    void topologicalSort(vector<int>indegreesCopy);
    void Dijkstra(int source,int aim,vector<int>&parents);
    bool isBipartite();
    bool Euler();
    void detectCycle();
    void BellmanFord(int start);
    void FloydWarshall();
    void PrimPath(int s,int aim);

};

void Graph::addEdgewithPair(int s, int u, int w) {
    pair <int,pair<int,int>>p;
    p.first=s;
    p.second.first=u;
    p.second.second=w;
    edges.push_back(p);
}

void Graph::addEdge(int s, int u,int weight) {  //directed
    adjacencyList[s][u]=weight;
    indegrees[u]++;
}
void Graph::addUndirectedEdge(int s,int u,int weight){
    adjacencyList[s][u]=weight;
    adjacencyList[u][s]=weight;
    indegrees[s]++; indegrees[u]++;
}

void Graph::addEdge(int s, int u){  //directed
    adjacencyList[s].push_back(u);
    indegrees[u]++;
}

void Graph::addUndirectedEdge(int s,int u){
    adjacencyList[s].push_back(u);
    adjacencyList[u].push_back(s);
    indegrees[s]++; indegrees[u]++;
}

void Graph::topologicalSort(vector<int>indegrees1){   //cycle condition check vs eksik
    queue<int>q;
    for(int i=0;i<numVertices;i++){
        if(indegrees1[i]==0){
            q.push(i);
            break;
        }
    }
    while(!q.empty()){
        int cur=q.front();
        q.pop();
        cout<<cur<<endl;
        for(int i=adjacencyList[cur].size()-1;i>=0;i--){
           int recent=adjacencyList[cur][i];
           indegrees1[recent]--;
           if(indegrees1[recent]==0){
               q.push(recent);
           }
        }
    }
}

void printPath(int aim,int source,vector<int>&parents){
    if(aim==source){
        cout<<aim<<" ";
        return;
    }

    printPath(aim,parents[source],parents);
    cout<<source<<" ";
}

void Graph::bfs(int s,int aim,vector<int>&parents) {  //shortest unweighted path
    vector<bool>visited(numVertices,false);  //or can make a distamces array, initiate with -1
    vector<int>distances(numVertices,-1);
    distances[s]=0;
    queue<int>q;
    q.push(s);
    visited[s]=1;
    while(!q.empty()){
        int cur=q.front();
        q.pop();

        if(cur==aim){
            cout<<"Distance: "<<distances[aim]<<endl;
            printPath(s,aim,parents);
            return;
        }
        for(int i=0;i<adjacencyList[cur].size();i++){
            int recent=adjacencyList[cur][i];
            if(visited[recent]!=1){
                q.push(recent);
                parents[recent]=cur;
                visited[recent]=1;
                distances[recent]=distances[parents[recent]]+1;
            }
        }
    }

}

void Graph::detectCycle(){
    vector<bool>visited(numVertices,false);
    vector<int>parent(numVertices,-1);
    queue<int>q;
    q.push(0);
    while(!q.empty()){
        int cur=q.front();
        q.pop();
        visited[cur]=1;
        for(int i=0;i<adjacencyList[cur].size();i++){
            if(visited[adjacencyList[cur][i]]){
                printPath(adjacencyList[cur][i],cur,parent);
                cout<<adjacencyList[cur][i]<<" ";
                return;
            }
            parent[adjacencyList[cur][i]]=cur;
            q.push(adjacencyList[cur][i]);
        }
    }
    cout<<"no cycle";
}

bool Graph::Euler(){

}


void Graph::dfs(int s){   //or recursive

    vector<int>visited(numVertices,false);
    stack<int>st;
    st.push(s);
    while(!st.empty()){
        int cur=st.top();
        st.pop();
        visited[cur]=1;
        cout<<cur<<" ";
        for(int i=0;i<adjacencyList[cur].size();i++){
            int recent=adjacencyList[cur][i];
            if(visited[recent]!=1){
                visited[recent]=1;
                st.push(recent);
            }
        }
    }


}


void Graph::dfsRecursive(int s,vector<bool>&visited){

    cout<<s<<" ";
    visited[s]=1;
    for(int i=0;i<adjacencyList[s].size();i++){
        if(!visited[adjacencyList[s][i]])
            dfsRecursive(adjacencyList[s][i],visited);
    }

}



void Graph::BellmanFord(int start) {

    vector<long long int>distances(numVertices,100000);
    vector<int>parents(numVertices,-1);
    parents[start]=0;
    distances[start]=0;

    for(int i=0;i<numVertices-1;i++){
        for(int i=0;i<edges.size();i++){
            int from=edges[i].first;
            int to=edges[i].second.first;
            long long int dist=edges[i].second.second;
            if(distances[from]+dist<distances[to]){
                distances[to]=distances[from]+dist;
                parents[to]=from;


            }
        }
    }
    for(int i=1;i<numVertices;i++){
        cout<<i<<": "<<distances[i]<<endl;
    }

}

void Graph::FloydWarshall(){

    vector<vector<int>>distances(numVertices+1);
    for(int i=0;i<numVertices;i++){
        distances[i].resize(numVertices+1);
        for (int j = 0; j <numVertices ; ++j) {
            distances[i][j]=100000;
        }
    }
    for (int k = 0; k < edges.size(); ++k) {
        int from=edges[k].first;
        int to=edges[k].second.first;
        int dist=edges[k].second.second;
        distances[from][to]=dist;
    }

    for (int k = 0; k < numVertices; ++k) {
        for (int i = 0; i <numVertices ; ++i) {
            for (int j = 0; j <numVertices ; ++j) {
                if (distances[i][j]>distances[i][k]+distances[k+1][j]){
                    distances[i][j]=distances[i][k]+distances[k+1][j];
                }
            }
        }
    }

    for(int i=0;i<numVertices;i++){
        for (int j = 0; j <numVertices ; ++j) {
            cout<<i<<"->"<<j<<": "<<distances[i][j];
        }
    }

}

void Graph::PrimPath(int s,int aim){

}

bool Graph::isBipartite(){   //has infinite loop but logic is true

    vector<int>color(numVertices,-1);
    color[0]=1;
    queue<int>q;
    q.push(0);
    while(!q.empty()){

        int cur=q.front();
        q.pop();
        for(int i=0;i<adjacencyList[cur].size();i++){
            if(color[adjacencyList[cur][i]==-1]){
                color[adjacencyList[cur][i]]=1-color[cur];
                q.push(adjacencyList[cur][i]);
            }
            else{
                if(color[cur]==color[adjacencyList[cur][i]]){
                    return false;
                }
            }
        }

    }
    return true;

}





int main() {









    return 0;
}