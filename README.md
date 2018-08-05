This implementation demostrates how to represent a  graph by using connectivity matrix in C++. 
In general, the graph density for an unordered graph is calculated as : density=numofEdges/maxnumOfEdges, where maxnumOfEdges = (vertices-1)*vertices/2. So in order to generate a random unordered graph with density 10% for example, we just need randomly select 10% of its edges and initialize it's edge value with a random number between 0.1 and 10.
Addtionally, the dijkstra's algorithm is implemented by using the C++ STL and iterator clases such as priority queue, unordered_map, vector and so on.
