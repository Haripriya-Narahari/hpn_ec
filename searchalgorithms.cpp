#include<iostream>
#include<stdlib.h>
#include <list>
#include <map>
#include<vector>
#include <algorithm>
#include <stack>
#include<queue>
using namespace std;
/*
Assumptions made:
1. Start node starts with 1
2. End node = Number of vertices
3. The cost is non-decreasing
*/
int graph[100][100]; list <int> path={}, final_path={}, best_path={}; int cost=0, len, num_vertices, num_edges, node1, node2, start, end_node;
int visited [100]; list <int> graph_cost, all_path;
vector<int> visited_nodes; 
int best_cost = INT_MAX;

int HeuristicCalculation(int start) {
    int best_cost = INT_MAX;
    queue<vector<int>> paths; 
    vector<int> initial_path = {start};
    vector<int> best_path;
    vector<vector<int>> all_paths; 
    int cost_arr[10];
    int g=0;
    paths.push(initial_path);

    while (!paths.empty()) {
        vector<int> current_path = paths.front();
        paths.pop();

        int current_node = current_path.back();

        if (current_path.size() > 1) {
            int current_cost = 0;
            for (size_t i = 0; i < current_path.size() - 1; i++) {
                int from = current_path[i];
                int to = current_path[i + 1];
            }
        }

        visited[current_node] = true;

        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0 && !visited[k]) {
                vector<int> new_path = current_path;
                new_path.push_back(k);
                paths.push(new_path);
                has_unvisited_neighbors = true;
            }
        }

        if (!has_unvisited_neighbors) {
            all_paths.push_back(current_path);
        }
    }

    int pathcost = 0;
    int min=INT_MAX;
    for (const vector<int>& path : all_paths) {
        pathcost = 0;
        for (int i = 0; i < path.size(); i++) {
            if (i < path.size() - 1) {
            }
            if(i!=0)
            {
                pathcost+=graph[path[i-1]][path[i]];
            }
        }
        if(min>pathcost)
        {
            min = pathcost;
        }
    }
    return min;
}
int calculateHeuristicCost(int node) {
    int h=0;
    if(graph[node][end_node]>0)
    {
        return graph[node][end_node];
    }
    else
    {
        h = HeuristicCalculation(start);
        h/= 2;
    }
    return h;
}

void BritishMuseumSearch(int start) {
    queue<vector<int>> paths; 
    vector<int> initial_path = {start};
    vector<vector<int>> all_paths; 

    paths.push(initial_path);

    while (!paths.empty()) {
        vector<int> current_path = paths.front();
        paths.pop();

        int current_node = current_path.back();

        if (current_path.size() > 1) {
            int current_cost = 0;
            for (int i = 0; i < current_path.size() - 1; i++) {
                int from = current_path[i];
                int to = current_path[i + 1];
                
            }

        }

        visited[current_node] = true;

        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0 && !visited[k]) {
                vector<int> new_path = current_path;
                new_path.push_back(k);
                paths.push(new_path);
                has_unvisited_neighbors = true;
            }
        }

        if (!has_unvisited_neighbors) {
            all_paths.push_back(current_path);
        }
    }

    cout << "All Paths Explored:" << endl;
    for (const vector<int>& path : all_paths) {
        for (size_t i = 0; i < path.size(); i++) {
            cout << path[i];
            if (i < path.size() - 1) {
                cout << " -> ";
            }
        }
        cout << endl;
    }
}


void DepthFirstSearch(int start)
{
    path.push_back(start);
    int i = start;
    visited[i] = 1;
    if (i == end_node) {
        for(int node : path) {
        std::cout << node << ", ";
    }
        return;
    }
    else{
        for(int k=1;k<=num_vertices;k++)
        {
            if(graph[i][k] >= 1 && visited[k]!=1)
            {
                i = k;
                DepthFirstSearch(k);
            }
            
        }
    }
    
}

void BreadthFirstSearch(int start)
{
    path.push_back(start);
    int i = start;
    visited[i] = 1;
    while(!path.empty())
    {
        int i = path.front();
        path.pop_front();
        final_path.push_back(i);
        
        for(int k=1;k<=num_vertices;k++)
        {
            if(graph[i][k] >= 1 && visited[k]!=1)
            {
                
                visited[k] = 1;
                path.push_back(k);
            }
            
        }
    }
    for(int node : final_path) 
    {
        std::cout << node << ", ";
    }              
    
}

void HillClimbing(int start)
{
    
    path.push_back(start);
    final_path.push_back(start);
    int i = start;
    visited[i] = 1;
    if (i == end_node) 
    {
        for(int node : final_path) 
        {
            std::cout << node << ", ";
        }
        std::cout<<"Cost:"<<cost;
        return;
    }
    else
    {
        int n =-1;
        int min = 100;
        for(int k=1;k<=num_vertices;k++)
        {
            if(graph[i][k]<=min && graph[i][k]!=0)
            {
                min = graph[i][k];
                n = k;
            }
            
        }
        if(visited[n]!=1)
        {
            cost+= graph[i][n];
            path.pop_back();
            HillClimbing(n);
            
        }
    }
      
}

void BeamSearch(int start, int beam=2)
{
    list<int>final_nodes;
    path.push_back(start);
    final_path.push_back(start);
    final_nodes.push_back(start);
    int cost = 0;
    int i = start;
    visited[i] = 1;
    int c=0;
    while(!path.empty())
    {
        
        int i = path.front();
        path.pop_front();
        if (i == end_node)
        {
            break;
        }
        int min = 10000;
        list <int> beam_nodes = {};
        for (int j=i+1;j<=num_vertices;j++)
        {
            graph_cost.push_back(graph[i][j]);
        }
        graph_cost.sort();

        while(beam_nodes.size()<=beam)
        {
            int min = graph_cost.front();
            graph_cost.pop_front();
            for(int n=i+1;n<=num_vertices;n++)
            {
                if(graph[i][n] == min)
                {
                    beam_nodes.push_back(n);
                    cost+=graph[i][n];
                }
                
            }
        }

        while(!beam_nodes.empty())
        {
            beam_nodes.sort();
            int node = beam_nodes.front();
            if(visited[node]!=1)
            {
                visited[node] = 1;
                path.push_back(node);
                final_path.push_back(node);
                cost+=graph[i][node];
                
            }
            beam_nodes.pop_front();
        }
    }

    cout<<"Order of visitng nodes:";
    for(int node : final_path) 
    {
        std::cout << node << ", ";
    }    
    std::cout<<"Cost:"<<cost;
    
}


void BranchAndBound(int start) {
    vector<int> path; 
    vector<int> initial_path = {start};
    vector<int> best_path;
    vector<vector<int>> all_paths; 
    int cost_arr[10];
    int g = 0;

    path.push_back(start);

    while (!path.empty()) {
        int current_node = path.back();

        visited[current_node] = true;

        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0 && !visited[k]) {
                path.push_back(k);
                has_unvisited_neighbors = false;
            }
        }

        if (!has_unvisited_neighbors) {
            int current_cost = 0;
            for (size_t i = 0; i < path.size() - 1; i++) {
                int from = path[i];
                int to = path[i + 1];
                current_cost += graph[from][to];
            }

            if (path.size() > 1) {
                cost_arr[g++] = current_cost;
            }
            
            all_paths.push_back(path);

            while (!path.empty() && visited[path.back()]) {
                path.pop_back();
            }
        }
    }

    int pathcost = 0;
    cout << "Paths Explored:" << endl;
    for (const vector<int>& path : all_paths) {
        pathcost = 0;
        for (size_t i = 0; i < path.size(); i++) {
            cout << path[i];
            if (i < path.size() - 1) {
                cout << " -> ";
            }
            if (i != 0) {
                pathcost += graph[path[i - 1]][path[i]];
            }
        }
        cout << "   Cost: " << pathcost;
        cout << endl;
    }
}

void BranchAndBoundExtList(int start) {
    int end = end_node;
    bool pathFound;
    vector<int> path; 
    vector<int> initial_path = {start};
    vector<int> best_path;
    vector<vector<int>> all_paths; 
    int cost_arr[10];
    int g = 0;

    path.push_back(start);

    while (!path.empty()) {
        int current_node = path.back();

        visited[current_node] = true;

        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0 && !visited[k]) {
                path.push_back(k);
                has_unvisited_neighbors = true;
            }
        }

        if (!has_unvisited_neighbors) {
            int current_cost = 0;
            for (int i = 0; i < path.size() - 1; i++) {
                int from = path[i];
                int to = path[i + 1];
                current_cost += graph[from][to];
            }

            if (path.size() > 1) {
                cost_arr[g++] = current_cost;
            }
            
            all_paths.push_back(path);

            while (!path.empty() && visited[path.back()]) {
                path.pop_back();
            }
        }
    }

    int pathcost = 0;
    cout << "Order of nodes visited for each possible branch:" << endl;
    for (const vector<int>& path : all_paths) {
        pathcost = 0;
        for (int i = 0; i < path.size(); i++) {
            cout << path[i];
            if (i < path.size() - 1) {
                cout << " , ";
            }
            if (i != 0) {
                pathcost += graph[path[i - 1]][path[i]];
            }
        }
        cout << "   Cost: " << pathcost;
        cout << endl;
    }
}



void OracleSearch(int start) {
    int best_cost = INT_MAX;
    queue<vector<int>> paths; 
    vector<int> initial_path = {start};
    vector<int> best_path;
    vector<vector<int>> all_paths; 
    int cost_arr[10];
    int g=0;

    paths.push(initial_path);

    while (!paths.empty()) {
        vector<int> current_path = paths.front();
        paths.pop();

        int current_node = current_path.back();

        if (current_path.size() > 1) {
            int current_cost = 0;
            for (int i = 0; i < current_path.size() - 1; i++) {
                int from = current_path[i];
                int to = current_path[i + 1];
            }
        }

        visited[current_node] = true;

        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0 && !visited[k]) {
                vector<int> new_path = current_path;
                new_path.push_back(k);
                paths.push(new_path);
                has_unvisited_neighbors = true;
            }
        }

        if (!has_unvisited_neighbors) {
            all_paths.push_back(current_path);
        }
    }

    int pathcost = 0;
    std::cout << "All Paths Explored:" << endl;
    for (const vector<int>& path : all_paths) {
        pathcost = 0;
        for (size_t i = 0; i < path.size(); i++) {
            std::cout << path[i];
            if (i < path.size() - 1) {
                std::cout << " -> ";
            }
            if(i!=0)
            {
                pathcost+=graph[path[i-1]][path[i]];
            }
        }
        std::cout<<"   Cost: "<<pathcost;
        std::cout << endl;
    }
}

void BranchAndBoundHeursitics(int start) {
    vector<int> path; 
    vector<int> initial_path = {start};
    vector<int> best_path;
    vector<vector<int>> all_paths; 
    int cost_arr[10];
    int g = 0;

    path.push_back(start);

    while (!path.empty()) {
        int current_node = path.back();

        visited[current_node] = true;

        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0 && !visited[k]) {
                path.push_back(k);
                has_unvisited_neighbors = false;
            }
        }

        if (!has_unvisited_neighbors) {
            int current_cost = 0;
            for (size_t i = 0; i < path.size() - 1; i++) {
                int from = path[i];
                int to = path[i + 1];
                int heuristic_cost = calculateHeuristicCost(to);
                current_cost += heuristic_cost;
            }

            if (path.size() > 1) {
                cost_arr[g++] = current_cost;
            }
            
            all_paths.push_back(path);

            while (!path.empty() && visited[path.back()]) {
                path.pop_back();
            }
        }
    }

    int pathcost = 0;
    cout << "Paths Explored:" << endl;
    for (const vector<int>& path : all_paths) {
        pathcost = 0;
        for (size_t i = 0; i < path.size(); i++) {
            cout << path[i];
            if (i < path.size() - 1) {
                cout << " -> ";
            }
            if (i != 0) {
                pathcost += graph[path[i - 1]][path[i]];
            }
        }
        cout << "   Cost: " << pathcost;
        cout << endl;
    }
}
void BranchAndBoundExtListHeuristics(int start) {
    vector<int> path; 
    vector<vector<int>> all_paths; 
    int cost_arr[10];
    int g = 0;

    path.push_back(start);
    int current_cost = 0; // Initialize the current cost to 0

    while (!path.empty()) {
        int current_node = path.back();

        visited[current_node] = true;

        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0 && !visited[k]) {
                path.push_back(k);
                has_unvisited_neighbors = true;
            }
        }

        if (!has_unvisited_neighbors) {
            // Calculate the heuristic cost for the current node
            int heuristic_cost = calculateHeuristicCost(current_node);

            // Update the current cost by adding the heuristic cost
            current_cost += heuristic_cost;

            if (path.size() > 1) {
                cost_arr[g++] = current_cost;
            }
            
            all_paths.push_back(path);

            while (!path.empty() && visited[path.back()]) {
                path.pop_back();
            }

            // Backtrack the heuristic cost when moving back in the path
            if (!path.empty()) {
                int previous_node = path.back();
                current_cost -= heuristic_cost;
                current_cost -= graph[previous_node][current_node];
            }
        }
    }

    int pathcost = 0;
    cout << "Order of nodes visited for each possible branch:" << endl;
    for (const vector<int>& path : all_paths) {
        pathcost = 0;
        for (int i = 0; i < path.size(); i++) {
            cout << path[i];
            if (i < path.size() - 1) {
                cout << " , ";
            }
            if (i != 0) {
                pathcost += graph[path[i - 1]][path[i]];
            }
        }
        cout << "   Cost: " << pathcost;
        cout << endl;
    }
}

void BestFirstSearch(int numVertices, int start, int end_node) {
    queue<pair<int, int>> q;
    vector<vector<int>> all_paths;
    vector<int> current_path;
    vector<int> best_path;
    int current_cost = 0;
    int best_cost = INT_MAX;
    q.push({ start, calculateHeuristicCost(start)}); 

    while (!q.empty()) {
        int current_node = q.front().first;
        int current_heuristic = calculateHeuristicCost(current_node);
        q.pop();

        current_path.push_back(current_node);

        if (current_node == end_node) {
            all_paths.push_back(current_path);
            break;
        }
        if (current_node == end_node && current_heuristic < best_cost) {
            // Update the best path and cost
            //best_path.clear();
            best_path.push_back(current_node);
            best_cost+= current_heuristic;
        }


        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= numVertices; k++) {
            if (graph[current_node][k] > 0) {
                int priority = calculateHeuristicCost(k);
                q.push({k, priority });
                has_unvisited_neighbors = true;
            }
        }

        if (!has_unvisited_neighbors) {
            if (!current_path.empty()) {
                current_path.pop_back();
            }
            if (current_heuristic < best_cost) {
                best_path.push_back(current_node);
                best_cost = current_heuristic;
            }
        }
    }
    int  cost=0;
    if (all_paths.empty()) {
        cout<<"No Path"; // No path found
    }
    else

    {   cout<<"Order of nodes visited:\n";
        for (int i =0;i<all_paths[0].size();i++)
        {
            cout<<all_paths[0][i]<<", ";
            if(i!=0)
            {for(int j =0;j<i;j++)
            {
                if(graph[all_paths[0][j]][all_paths[0][i]]>0)
                {
                    cost+=graph[all_paths[0][j]][all_paths[0][i]];
                    break;
                }
            }
            }
        }

        cout << "\nCost of the Best Path: " <<cost;

    }
    
}

void AStar(int start, int end_node) {
    queue<pair<int, int>> q;
    vector<vector<int>> all_paths;
    vector<int> current_path;
    vector<int> best_path;
    int current_cost = 0;
    int best_cost = INT_MAX;
    q.push({ start, calculateHeuristicCost(start)}); 

    while (!q.empty()) {
        int current_node = q.front().first;
        int current_heuristic = calculateHeuristicCost(current_node);
        q.pop();

        current_path.push_back(current_node);

        if (current_node == end_node) {
            all_paths.push_back(current_path);
            break;
        }
        if (current_node == end_node && current_heuristic < best_cost) {
            // Update the best path and cost
            //best_path.clear();
            best_path.push_back(current_node);
            best_cost+= current_heuristic;
        }


        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0) {
                int priority = calculateHeuristicCost(k)+graph[current_node][k];
                q.push({k, priority });
                has_unvisited_neighbors = true;
            }
        }

        if (!has_unvisited_neighbors) {
            if (!current_path.empty()) {
                current_path.pop_back();
            }
            if (current_heuristic < best_cost) {
                best_path.push_back(current_node);
                best_cost = current_heuristic;
            }
        }
    }
    int  cost=0;
    if (all_paths.empty()) {
        cout<<"No Path"; // No path found
    }
    else

    {   cout<<"Order of nodes visited:\n";
        for (int i =0;i<all_paths[0].size();i++)
        {
            cout<<all_paths[0][i]<<", ";
            if(i!=0)
            {for(int j =0;j<i;j++)
            {
                if(graph[all_paths[0][j]][all_paths[0][i]]>0)
                {
                    cost+=graph[all_paths[0][j]][all_paths[0][i]];
                    break;
                }
            }
            }
        }

        cout << "\nCost of the Best Path: " <<cost;

    }
}
//To be done:Solve issue of no path return for any graph in certain cases
void AOStar(int start, int end_node) {
    queue<pair<int, int>> q;
    vector<vector<int>> all_paths;
    vector<int> current_path;
    vector<int> best_path;
    int current_cost = 0;
    int best_cost = INT_MAX;
    q.push({ start, calculateHeuristicCost(start)}); 

    while (!q.empty()) {
        int current_node = q.front().first;
        int current_heuristic = calculateHeuristicCost(current_node);
        q.pop();

        current_path.push_back(current_node);

        if (current_node == end_node) {
            all_paths.push_back(current_path);
            break;
        }
        if (current_node == end_node && current_heuristic < best_cost) {
            best_path.push_back(current_node);
            best_cost+= current_heuristic;
        }


        bool has_unvisited_neighbors = false;

        for (int k = 1; k <= num_vertices; k++) {
            if (graph[current_node][k] > 0) {
                int priority = calculateHeuristicCost(k)+graph[current_node][k];
                q.push({k, priority });
                has_unvisited_neighbors = true;
            }
        }

        if (!has_unvisited_neighbors) {
            if (!current_path.empty()) {
                current_path.pop_back();
            }
            if (current_heuristic < best_cost) {
                best_path.push_back(current_node);
                best_cost = current_heuristic;
            }
        }
    }
    int  cost=0;
    if (all_paths.empty()) {
        cout<<"No Path"; 
    }
    else

    {   cout<<"Order of nodes visited:\n";
        for (int i =0;i<all_paths[0].size();i++)
        {
            cout<<all_paths[0][i]<<", ";
            if(i!=0)
            {for(int j =0;j<i;j++)
            {
                if(graph[all_paths[0][j]][all_paths[0][i]]>0)
                {
                    cost+=graph[all_paths[0][j]][all_paths[0][i]];
                    break;
                }
            }
            }
        }

        cout << "\nCost of the Best Path: " <<cost;

    }
}



void main()
{
    std::cout<<"Enter number of vertices:";
    cin>>num_vertices;
    std::cout<<"Enter number of edges:";
    cin>>num_edges;
    std::cout<<"Enter source vertex:";
    cin>>start;
    std::cout<<"Enter destination vertex:";
    cin>>end_node;
    for (int i = 0; i <= num_vertices; i++) {
        for (int j = 0; j <= num_vertices; j++) {
            graph[i][j] = 0;
        }
    }
    std::cout<<"Enter edges of a graph and cost: (Eg: node1 node2 cost) ";
    for (int k=0; k<num_edges; k++)
    {
        cin>>node1>>node2>>len;
        graph[node1][node2] = len;
    }

    int choice=-1;
    cout<<"Enter choice for algorithm:\n1.British Museum Search\n2.Depth First Search\n3.Breadth First Search\n4.Hill Climbing";
    cout<<"\n5.Beam Search\n6.Oracle\n7.Branch And Bound\n8.Branch And Bound Extended List\n9.Branch And Bound - Heuristics";
    cout<<"\n10.Branch And Bound Extended List- Heuristics\n11.Best First Search\n12.AO Star\n13.A Star";
    cin>>choice;
    switch(choice)
    {
    case 1: BritishMuseumSearch(start); break;
    case 6: OracleSearch(start);break;
    case 2: DepthFirstSearch(start); break;
    case 3: BreadthFirstSearch(start);break;
    case 4: HillClimbing(start);break;
    case 5: BeamSearch(start);break;
    case 7: BranchAndBound(start);break;
    case 8: BranchAndBoundExtList(start);break;
    case 9: BranchAndBoundHeursitics(start);break;
    default: break;
    }
    if(choice>9)
    {
        if (choice==10)
        {
            BranchAndBoundExtListHeuristics(start);
        }
        else if(choice==11)
        {
            BestFirstSearch(num_vertices,start,end_node);
        }
        else if(choice==11)
        {
            AOStar(start,end_node);
        }
        else if (choice==12)
        {
            AStar(start,end_node);
        }
    }

    
}