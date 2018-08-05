/******************************************************************************

                              Online C++ Compiler.
               Code, Compile, Run and Debug C++ program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/
// Example program
#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <map>
#include <limits>
#include <unordered_map>
#include <algorithm>
#include <numeric>


using namespace std;

#define FloatMax  (numeric_limits<float>::max())  // define the maximum value for float type

typedef pair<int, float> Pair;  // set node and vertices as a pair


// overloading compare operator in the priority_queue
struct Compare {
    constexpr bool operator()(pair<int, float> const & a,
                              pair<int, float> const & b) const noexcept
    { return a.second>b.second; }
};

// class for generating random graph matrix
class graph{
    private:
        int vertices;  // number of vertices or nodes
        int edges;    // number of edges
        float **graph_ptr;  // generating the graph as a connectivity matrix
    public:
        graph(int vertices, float dense, float low, float high); // constructor
        ~graph();      // destructor
        int numVertice(void);  // get the number of vertices
        int numEdge(void); // get the number of edges
        bool adjacent (float **G, int x, int y); // tests whether there is an edge from node x to node y.
        unordered_map<int,float>neighbors (float **G, int x); // lists all nodes y such that there is an edge from x to y.
        float get_node_value (float**G,int x); // returns the value associated with the node x.
        void set_node_value( float**G, int x, float a); // sets the value associated with the node x to a.
        float get_edge_value(float **G, int x, int y); // returns the value associated to the edge (x,y).
        void set_edge_value (float **G, int x, int y, float v); // sets the value associated to the edge (x,y) to v.
        float** getGraphMatrix(void);  // get the pointer to the graph matrix
        void display(void); // display the graph matrix in the console
     
};

graph::graph(int vertices, float dense, float low, float high)
{
    int i=0,j = 0,edg = 0;    
    this->edges = (int)(dense*vertices*(vertices-1)/2); // get the number of edges
    this->vertices = vertices;
    
    // allocate memory for the graph matrix
    this->graph_ptr = new float*[vertices];
    for (int i= 0; i < vertices; i++)
    {
        this->graph_ptr[i] = new float[vertices];
        for(int j = 0; j < vertices; j++)
        {
            graph_ptr[i][j] = FloatMax;
        }
    }
    
    /*  random graph procedure which means:
        if dense = 10%, then the graph would have 10% of its edges picked at random 
        and its edge distance would be selected at random from the distance range
    */
    while(edg<edges)
    {
       i = rand()%this->vertices;
       j = rand()%this->vertices;
       if(i!=j && this->graph_ptr[i][j] == FloatMax)
       {
           float randV = low + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(high-low)));
           this->set_edge_value(this->graph_ptr,i,j,randV);
           this->set_edge_value(this->graph_ptr,j,i,randV);
           edg++;
        }
    }   
}


graph::~graph()
{
    int i;
    for (int i= 0; i < this->vertices; i++) 
    {
        delete graph_ptr[i];   // delete the allocated graph matrix
    }
}

// get the number of vertices
int graph::numVertice(void)
{
    return this->vertices;
}

// get the number of edges
int graph::numEdge(void)
{
    return this->edges;
}

// lists all nodes y such that there is an edge from x to y.
unordered_map<int,float> graph::neighbors (float **G, int x)
{
    int i;
    unordered_map<int, float> neighbours;
    for(i=0;i<this->vertices;i++)
    {
        if(this->adjacent(G,x,i))
        {
            neighbours.insert ( std::pair<int,float>(i,G[x][i]) );
        }
    }
    return neighbours;
}

// tests whether there is an edge from node x to node y.
bool graph::adjacent (float **G, int x, int y)
{
    if(x!=y && G[x][y]!=FloatMax)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// sets the edge value between node x and node y to v
void graph::set_edge_value(float **G,int x, int y, float v)
{
    G[x][y] = v;
}

// get the edge value between node x and node y
float graph::get_edge_value(float **G,int x, int y)
{
    return G[x][y];
}

// get the node value of node x
float graph::get_node_value (float**G,int x)
{
    return G[x][x];    
}

// sets the value associated with the node x to a.
void graph::set_node_value( float**G, int x, float a)
{
    G[x][x] = a;    
}

// get the graph matrix
float** graph::getGraphMatrix(void)
{
    
    return this->graph_ptr;
}

// output the graph matrix on the console
 void graph::display(void)
{
    int i,j;
    for(i = 0;i < this->vertices;i++)
    {
        for(j = 0; j < this->vertices; j++)
            cout<<graph_ptr[i][j]<<"  ";
        cout<<endl;
    }
}


// class for getting the average shortest path from node v to the rest nodes
class ShortestPath
{
   private:
       graph *graph_ptr;
       float **graph_matrix_ptr;
       unordered_map<int, vector<int>> path;    // the shortest path from srcNode to the rest of nodes
       vector<float> distance;  // the shortest path from srcNode to the rest of nodes
       float averPathCost;   // the averaage shortest path between v and the rest of nodes
       int numOfVertices;   // number of nodes  
   public:
       ShortestPath(graph* g_ptr);
       void pathGen(int src);  // getting the shortest path and path cost from node src to the rest of nodes
       float path_size(int v, int w); // return the shortest path size between node v and node w
       float aver_path_size(int v);  // return the average shortest path between node v and the rest of nodes
       void display_short_path(int v, int w); // display the shortest path between node v and node w on the console
       
};


ShortestPath::ShortestPath(graph* g_ptr)
{
    this->averPathCost=0;
    this->graph_ptr = g_ptr;
    this->averPathCost = 0;
    this->graph_matrix_ptr = this->graph_ptr->getGraphMatrix();
    this->numOfVertices = this->graph_ptr->numVertice();
    for(int i=0;i<this->numOfVertices;i++)
    {
        this->distance.push_back(FloatMax); // the shortest path from srcNode to the rest of nodes
    }
}

void ShortestPath::pathGen(int src)
{
    // getting the matrix index of the source node
    // example, if the source node is 1, then the matrix index would be 0.
    int srcNode = src-1, curNode = src-1, curNeighbor;
    
    float accWeight = 0;
    
    unordered_map<int,float> neighbours; 
    vector<int> visitedNode;
    priority_queue<pair<int,float>, vector<pair<int,float>>, Compare > q;
    
    q.push(make_pair(srcNode, 0));  // add the source node as the initial element in the priority_queue
    
    while(!q.empty())
    {
        curNode = q.top().first;
        accWeight = q.top().second;
        
        q.pop();
        neighbours = this->graph_ptr->neighbors(this->graph_matrix_ptr,curNode);

        for (unordered_map<int,float>::iterator it=neighbours.begin(); it!=neighbours.end(); ++it)
        {
            std::vector<int>::iterator it_vec;
            it_vec = find (visitedNode.begin(), visitedNode.end(), it->first);
            // if the node hasn't been visiied
            if (it_vec == visitedNode.end())
            {
                 curNeighbor = it->first;
                 float weight = it->second;
                 if(distance.at(curNeighbor) > accWeight  + weight)
                 {
                     // updating smallest distance of the next node
                     this->distance.at(curNeighbor) = accWeight + weight;
                     q.push(make_pair(it->first,this->distance.at(curNeighbor)));
                     
                     // updating the path from srcNode to the current node
                     //this->path[curNeighbor].insert(this->path[curNeighbor].begin(), this->path[curNode].begin(),this->path[curNode].end());
                     if(distance.at(curNeighbor) != FloatMax && this->path[curNeighbor].size()!=0)
                     {
                         this->path[curNeighbor].pop_back();
                     }
                     this->path[curNeighbor].push_back(curNode);
                     it_vec = find (this->path[curNeighbor].begin(), this->path[curNeighbor].end(), curNeighbor);
                     if(it_vec == path[curNeighbor].end())
                     {
                        path[curNeighbor].push_back(curNeighbor);
                     }
                        
                 }
                 
            }
       }
       
       visitedNode.push_back(curNode);
    }

    for(int i=0;i<this->numOfVertices;i++)
    {
        // there is no path between the source and the other node, set the edge between the two nodes as 0
       if(this->distance.at(i) == FloatMax)
       {
           this->distance.at(i) = 0;
       }
       
    }
}

// return the average shortest path between node v and the rest of nodes
float ShortestPath::aver_path_size(int v)
{
    this->pathGen(v);
    
    this->averPathCost = accumulate( this->distance.begin(), this->distance.end(), 0.0)/this->distance.size();
    
    cout<<"\n"<<"the average shortest path between node "<<v<<" and the rest nodes is:\n";
    cout<< this->averPathCost;
    
    return this->averPathCost;
    
}

 // return the shortest path size between node v and node w
float ShortestPath::path_size(int v, int w)
{
    this->pathGen(v);
    
    cout<<"\n"<<"the shortest path cost between node "<<v<<" and node "<<w<<" is:\n";
    cout<<this->distance.at(w-1);
    return this->distance.at(w-1);
}

// display the shortest path between node v and node w on the console
void ShortestPath::display_short_path(int v, int w)
{
    this->pathGen(v);
    cout<<"\n"<<"the shortest path between node "<<v<<" and node "<<w<<" is:\n";
    for(vector<int>::iterator i = this->path[w-1].begin(); i != this->path[w-1].end(); ++i)
    {
        cout<<" "<<*i+1<<" ";
    }
}


int main()
{
    
  cout<<" In this excersize, I've understood how to represent a  graph by using connectivity matrix in C++. I've better understood the relationship between graph\
  density and graph vertices. In general, the graph density for an unordered graph is calculated as : density=numofEdges/maxnumOfEdges, where \
  maxnumOfEdges = (vertices-1)*vertices/2. So in order to generate a random unordered graph with density 10% for example, we just need randomly select 10% of its edges\
  and initialize it's edge value with a random number between 0.1 and 10. \
  Addtionally, I've learned how to use the dijkstra's algorithm to calculate the shortest path between two nodes in a graph. I've also been able to implement this \
  algorthim by using the C++ STL and iterator clases such as priority queue, unordered_map, vector and so on.\n";
  // case 1
  cout<<"\n case 1: vertice is 50 and graph density is 20'%': \n";
  int vertices = 50;
  float dense = 0.2;
  float distRangMin = 0.1;
  float distRangMax = 10;
  graph graph1(vertices,dense,distRangMin,distRangMax);
  // graph1.display();
  graph *graph_ptr;
  graph_ptr = &graph1;
  ShortestPath shortestpath(graph_ptr);
  shortestpath.display_short_path(1,50);
  shortestpath.path_size(1,50);
  shortestpath.aver_path_size(1);
   
  // case 2
  cout<<"\n case 2: vertice is 50 and graph density is 40'%': \n";
  vertices = 50;
  dense = 0.4;
  distRangMin = 0.1;
  distRangMax = 10;
  graph graph2(vertices,dense,distRangMin,distRangMax);
  // graph2.display();
  graph_ptr = &graph2;
  ShortestPath shortestpath2(graph_ptr);
  shortestpath2.display_short_path(2,50);
  shortestpath2.path_size(2,50);
  shortestpath2.aver_path_size(1);

 }


