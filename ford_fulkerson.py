##########################################################################################################
# 1. Fast Backups
##########################################################################################################
from queue import Queue
class Graph:
    """
    Refer to the Malaysia Week04 and Week05 Lecture Video explained by Mr.Lim Wern Han
    I reused this from my assignment 1
    """
    def __init__(self, V, E, maxIn, maxOut) -> None:
        """
		Function description: This function creates each vertex using verteces input, and add edges inside each vertex accordingly.

		Approach description: Since I need to create graph to do dijstra, I made a graph using vertices and edges which I got from input. 
		
        This function loop for length of vertices and length of edges so time complexity is
		Input: 
			V: an array of vertices
            E: an array of edges
            reverse: a boolean whether the graph is reverse or not.
		
		Output: This function does not return anything.
		
		Time complexity: O(V + E), where V is the number of vertices and E is the number of edges.
		Aux space complexity: O(V + E), where V is the number of vertices and E is the number of edges.
        """
        
        # make vertices and insert into graph class variable
        self.vertices = [None] * (len(V))
        for i in range(len(V)):
            self.vertices[i] = Vertex(V[i])
            self.vertices[i].maxIn = maxIn[i]
            self.vertices[i].maxOut = maxOut[i]
            
        # make edges and insert in to graph class variable
        for i in range(len(E)):
            edge_info = E[i]
            self.vertices[edge_info[0]].edges.append(Edges(edge_info[0], edge_info[1], edge_info[2]))
                   

class Vertex:
    """
    Refer to the Malaysia Week04 and Week05 Lecture Video explained by Mr.Lim Wern Han
    I reused this from my assignment 1
    """
    def __init__(self, id) -> None:
        """
		Function description: This function initilizes vertex by creating id, edges, discovered, distance, previous, visited information.

		Approach description: Each vertex should have id, list of edges, discovered boolean, distance, previous vertex, visited boolean.
		
		Input: This function does not have input
		
		Output: This function does not return anything.
		
		Time complexity: O(1)
		Aux space complexity: O(E), where E is the number of edges.
  
        """
        # vertex class can have id, edges, discovered, distance, previous and visited
        self.id = id
        self.edges = []
        self.discovered = False
        self.distance = 0
        self.previous = None
        self.visited = False
        self.passable = True
        self.reachable = True
        self.maxIn = 0
        self.maxOut = 0

class Edges:
    """
    Refer to the Malaysia Week04 and Week05 Lecture Video explained by Mr.Lim Wern Han
    I reused this from my assignment 1
    """
    def __init__(self, u, v, w) -> None:
        """
		Function description: This function initilizes edge by creating u(starting vertex of the edge), v(ending vertex of the edge), w(weight of the edge)

		Approach description: Each edge should have u, v, w.
		
		Input: This function does not have input
		
		Output: This function does not return anything.
		
		Time complexity: O(1)
		Aux space complexity: O(1)
  
        """
        # edge class can have u, v and w
        self.u = u
        self.v = v
        self.w = w
class Path():
    '''
    This class saves path which is a list of order of a vertex. It saves also residual capacity which is the minimum capacity of the path
    '''
    def __init__(self, path, residual_capacity) -> None:
        """
		Function description: This function initilizes path creating by path and residual_capacity.

		Approach description: Each path should have path and residual_capacity
		
		Input: This function does not have input
		
		Output: This function does not return anything.
		
		Time complexity: O(1)
		Aux space complexity: O(1)
  
        """
        # path class can have path and residual_capacity
        self.path = path
        self.residual_capacity = residual_capacity
              
class ResidualNetwork():
    '''
    I refer to the Malaysia week8 Lecture Video explained by Mr.Lim Wern Han
    '''
    def __init__(self, graph, origin, targets) -> None:
        """
		Function description: This function initilizes ResidualNetwork by creating graph, origin, sink, vertices for sink and connect targets and sink

		Approach description: Each path should have path and residual_capacity
		
		Input: 
            graph: normal graph which is made by Graph class
            origin: the point to start
            targets: the point where data should be backed up
		
		Output: This function does not return anything.
		
		Time complexity: O(T), where T is targets.
		Aux space complexity: O(T), where T is targets
  
        """
        # ResidualNetwork class can have graph, origin  and sink, and make vertex for sink
        # sink is the vertex which is get edges from targets
        self.graph = graph
        self.origin = origin
        self.sink = len(graph.vertices)
        graph.vertices.append(Vertex(self.sink))
        
        # make edges from targets to sink, and change target maxOut with maxIn. And add the max out of targets to maxIn_for_sink.
        maxIn_for_sink = 0
        # for loop for targets
        for target in targets:
            # get target vertex by using target
            target_vertex = graph.vertices[target]
            #change target maxOut with maxIn, this is because from target to sink there is no maxOut
            target_vertex.maxOut = target_vertex.maxIn 
            # create edge from target to sink
            target_vertex.edges.append(Edges(target, self.sink, target_vertex.maxOut))
            maxIn_for_sink += target_vertex.maxOut
        
        # Change maxIn of sink with maxIn_for_sink.
        graph.vertices[self.sink].maxIn = maxIn_for_sink
          
    def has_AugmentingPath(self):
        """
		Function description: This function checks whether there is a path from origin to sink.

		Approach description: Do a bfs and if the vertex which is poped is sink then return true, otherwise return false
		
		Input: This function does not have any inputs
		
		Output: The output is boolean which shows whether there is a path or not.
		
		Time complexity: O(V + E), where V is the number of vertices and E is the number of edges..
		Aux space complexity: O(V + E), where V is the number of vertices and E is the number of edges.
  
        """
        # do bfs until the the vertex which is poped is sink
        discovered = Queue()
        discovered.put(self.origin)
        # while length of discovered is bigger than 0, loop
        while discovered.qsize() > 0:
            # pop top value from discovered and get vertex
            u_id = discovered.get()
            u = self.graph.vertices[u_id]
            u.visited = True
            # if the id of vertex is same as sink, return true
            if u_id == self.sink:
                return True
            # for loop for the edges of u
            for edge in u.edges:
                v_id = edge.v
                v = self.graph.vertices[v_id]
                # if v.discovered is False, change v.discovered is true and add v to discovered, and v.previous is id of u
                if v.discovered == False and v.reachable == True:
                    discovered.put(v_id)
                    v.discovered = True
        # if the sink is not reachable, return false
        return False
    
    def get_AugmentingPath(self):
        """
		Function description: This function returns the path from origin to sink and residual capacity.

		Approach description: Get a path by doing bfs, and using path get residual capacity by checking edge weight, maxIn and maxOut.
		
		Input: This function does not have any inputs
		
		Output: The output is Path class.
		
		Time complexity: O(V + E), where V is the number of vertices and E is the number of edges..
		Aux space complexity: O(V + E), where V is the number of vertices and E is the number of edges.
  
        """
        for vertex in self.graph.vertices:
            vertex.visited = False
            vertex.discovered = False
            vertex.previous = None
        # make path and discovered list and append origin to discovered
        path = []
        discovered = Queue()
        discovered.put(self.origin)
        # while length of discovered is bigger than 0
        while discovered.qsize() > 0:
            # pop the top value and get vertex u by using u_id. and make u.visited true
            u_id = discovered.get()
            u = self.graph.vertices[u_id]
            u.visited = True
            # loop for edges of u
            for edge in u.edges:
                v_id = edge.v
                v = self.graph.vertices[v_id]
                # if v.discovered is False, change v.discovered is true and add v to discovered, and v.previous is id of u
                if v.discovered == False and v.reachable == True:
                    discovered.put(v_id)
                    v.discovered = True
                    v.previous = edge
        
        # print("@@@@@@@@@@@@@@@@@")
        # set previous as previous of sink vertex and append sink to path
        vertices = self.graph.vertices
        previous = vertices[self.sink].previous
        # while previous is not origin
        while previous.u != self.origin:
            # append previous to path and change previous with previous of previous vertex
            path.append(previous)
            previous = vertices[previous.u].previous
        # append origin to path
        path.append(vertices[previous.v].previous)
        # reverse 
        path.reverse()
        way = []
        for kl in path:
            way.append(kl.u)
            
        # print(way)


        
        # set infinity to residual_capacity
        residual_capacity = float("inf")
        # for loop for range of path length - 1
        for i in range(len(path) - 1):
            # get the vertex of the path which is positioning i in path
            # if maxOut is smaller than residual_capacity, the residual_capacity become max_out
            max_out = vertices[path[i].u].maxOut
            if max_out < residual_capacity:
                residual_capacity = max_out
            # if maxIn is smaller than residual_capacity, the residual_capacity become max_in
            max_in = vertices[path[i].v].maxIn
            if max_in < residual_capacity:
                residual_capacity = max_in
        # for loop for edges of path_vertex
        for edge in path:
            # if max_throughput is smaller than residual_capacity, residual_capacity become max_throughput
            max_throughput = edge.w
            if max_throughput < residual_capacity:
                residual_capacity = max_throughput
            

        # create Path class with path and residual_capacity
        return Path(path, residual_capacity)
    
    def augmentFlow(self, path):
        """
		Function description: This function updates the path with path..residual_capacity and path.path

		Approach description: deduct residual_capacity from maxOut, edge weight and max_in of every vertex in path
		
		Input: 
            path: The list of path vertex
		
		Output: This function does not return anything
		
		Time complexity: O(P + E), where P is the number of vertices in path list and E is the number of edges.
		Aux space complexity: O(P + E), where P is the number of vertices in path list and E is the number of edges.
  
        """
        # make residual_capacity and this_path with path.residual_capacity and path.path
        residual_capacity = path.residual_capacity
        # print("residual capacity: " + str(residual_capacity))
        this_path = path.path
        vertices = self.graph.vertices
        # looping for range of length of this_path - 1
        for i in range(len(this_path) - 1):
            # get path_vertex with this_path[i]
            # minus residual_capacity from maxOut
            vertices[this_path[i].u].maxOut -= residual_capacity
            # minus residual_capacity from maxIN of the vertex of next path.
            vertices[this_path[i].v].maxIn -= residual_capacity
            # if maxOut or maxIn is 0, then remove every edges in path_vertex
            if vertices[this_path[i].u].maxOut == 0:
                vertices[this_path[i].u].reachable = False
            elif vertices[this_path[i].v].maxIn == 0:
                vertices[this_path[i].v].reachable = False
                
        # for loop for edges of path_vertex
        for edge in this_path:
            # minus residaul_capacity from edge weight
            edge.w -= residual_capacity
            # if edge weight become 0, remove the edge from edges in path_vertex
            if edge.w == 0:
                for vertex_edge in vertices[edge.u].edges:
                    if vertex_edge.v == edge.v:
                        vertices[edge.u].edges.remove(vertex_edge)

        # reset every visited and discovered to false
        for vertex in self.graph.vertices:
            vertex.visited = False
            vertex.discovered = False
            
            
            

             
    

def ford_fulkerson(graph, origin, targets):
    """
    I refer to the Malaysia week8 Lecture Video explained by Mr.Lim Wern Han
	Function description: This function return the maximum possible data throughput from the data centre origin to the data centres specified in targets.
	Approach description: while there is augmenting path, do residual_network.get_AugmentingPath and add path.residual_capacity to flow. Finaly do residual_network.augmentFlow.
	
    While loop can be called F times, where F is the maximum flow. Time complexity of has_AugmentingPath() is O(V + E), Time complexity of get_AugmentingPath() is O(V + E),
    Time complexity of augmentFlow() is O(P + E). Thus, time complexity in total will be O(F(V + E + V + E + P + E)) = O(F(2V + P + 3E))
    = O(2FV + FP + 3FE). Since P < V < E, I can say the time complexity is O(3FE) = O(FE)
    
	Input: 
        graph: normal graph which is made by Graph class
        origin: the point to start
        targets: the point where data should be backed up
	
	Output: This function returns the maximum possible data throughput from the data centre origin to the data centres specified in targets.
 
 	Time complexity: O(FE), where F is the maximum flow, and E is the number of edges.
	Aux space complexity: O(FE), where F is the maximum flow, and E is the number of edges.
    """
    # initilize flow
    flow = 0
    # initilize the residual network
    residual_network = ResidualNetwork(graph, origin, targets)

    count = 1
    # as long as there is an agumenting path
    while residual_network.has_AugmentingPath():
        # print("##########################################3")
        # print(count)
        count += 1
        # take the path
        path = residual_network.get_AugmentingPath()
        # augment the flow equal to the residual capacity
        flow += path.residual_capacity
        # updating the residual network
        residual_network.augmentFlow(path)
    return flow

def maxThroughput(connections, maxIn, maxOut, origin, targets):
    """
	Function description: This function return the maximum possible data throughput from the data centre origin to the data centres specified in targets.
	Approach description: Create Graph using vertices and connections and maxIn and MaxOut, use ford_fulkerson.
	
    This function do looping fo connections and max_vertex, so it takes O(E + V) where E is the number of edges and V is the number of vertices
    Since ford_fulkerson use O(FE) time complexity, total time complexity is O(E + V + FE) = O(FE)
	Input: 
        connections: direct communication channels between the data centres
        maxIn: list of integers in which maxIn[i] specifies the maximum amount of incoming data that data centre i can process per second
        maxOut: list of integers in which maxOut[i] specifies the maximum amount of outgoing data that data centre i can process per second
        origin: the point to start
        targets: the point where data should be backed up
	
	Output: This function does not return anything
	
	Time complexity: O(FE), where F is the maximum flow, and E is the number of edges.
	Aux space complexity: O(FE), where F is the maximum flow, and E is the number of edges.

    """
    # find max_vertex by looping roads
    max_vertex = 0
    for connection in connections:
        if connection[0] > max_vertex:
            max_vertex = connection[0]
        if connection[1] > max_vertex:
            max_vertex = connection[1]
    # create vertice from 0 to max_vertex
    vertices = []
    for i in range(max_vertex+1):
        vertices.append(i)
    graph = Graph(V=vertices, E=connections, maxIn=maxIn, maxOut=maxOut)
    return ford_fulkerson(graph, origin, targets)

if __name__ == "__main__":
    # connections = [(21, 48, 116), (35, 33, 476), (12, 21, 302), (34, 4, 179), (12, 36, 458), (47, 3, 438), (42, 40, 384), (3, 5, 347), (40, 27, 320), (43, 32, 340), (30, 17, 309), (48, 27, 244), (36, 26, 309), (19, 2, 359), (42, 33, 90), (26, 44, 86), (11, 26, 273), (43, 24, 478), (23, 1, 420), (0, 1, 202), (24, 37, 116), (34, 15, 143), (23, 14, 466), (29, 23, 338), (32, 26, 232), (9, 6, 296), (39, 3, 253), (47, 27, 289), (8, 21, 186), (32, 39, 488), (10, 0, 441), (31, 28, 92), (8, 26, 221), (33, 41, 234), (16, 22, 100), (37, 35, 489), (42, 43, 311), (22, 20, 77), (45, 8, 105), (7, 9, 431), (8, 35, 336), (5, 33, 487), (35, 39, 358), (21, 34, 456), (43, 38, 92), (32, 1, 191), (4, 47, 391), (41, 20, 257), (28, 12, 293), (27, 14, 403), (14, 46, 420), (19, 29, 94), (15, 48, 123), (39, 45, 85), (2, 13, 444), (44, 46, 461), (14, 3, 424), (20, 11, 75), (46, 16, 98), (17, 8, 464), (46, 7, 214), (27, 24, 217), (17, 35, 490), (47, 26, 173), (36, 9, 424), (29, 42, 471), (43, 15, 353), (22, 9, 119), (32, 18, 186), (39, 9, 459), (13, 26, 205), (48, 12, 238), (38, 26, 437), (10, 47, 77), (1, 17, 322), (44, 42, 88), (18, 35, 214), (7, 18, 477), (14, 11, 144), (25, 8, 410), (6, 19, 149), (1, 46, 165), (35, 20, 284), (27, 19, 311), (48, 35, 116), (18, 17, 494), (36, 48, 180), (37, 0, 219)]
    # maxIn = [747, 608, 848, 781, 793, 878, 642, 523, 568, 772, 578, 785, 585, 654, 603, 532, 913, 832, 629, 867, 911, 759, 615, 807, 784, 616, 552, 731, 942, 602, 505, 570, 749, 565, 833, 884, 669, 552, 707, 667, 549, 613, 798, 807, 809, 611, 762, 955, 898]
    # maxOut = [765, 694, 673, 595, 739, 636, 641, 769, 665, 779, 526, 569, 600, 612, 667, 752, 725, 522, 744, 539, 776, 576, 522, 740, 569, 504, 611, 618, 719, 542, 697, 643, 759, 666, 675, 621, 697, 597, 601, 597, 704, 712, 753, 622, 632, 721, 763, 694, 695]
    # origin = 35
    # targets = [7, 29, 31]
    
    
    # connections = [(0, 1, 3000), (1, 2, 2000), (1, 3, 1000),
    # (0, 3, 2000), (3, 4, 2000), (3, 2, 1000)]
    # maxIn = [5000, 3000, 3000, 3000, 2000]
    # maxOut = [5000, 3000, 3000, 2500, 1500]
    # origin = 0
    # targets = [4, 2]

    connections = [(2, 4, 4), (4, 0, 11), (2, 1, 5), (4, 3, 8), (1, 0, 7), (3, 2, 9)]
    maxIn = [2, 15, 12, 13, 3]
    maxOut = [7, 10, 6, 9, 3]
    origin = 2
    targets = [0, 3]
    expected = 5
    print(maxThroughput(connections, maxIn, maxOut, origin, targets))
    
    # # find max_vertex by looping roads
    # max_vertex = 0
    # for connection in connections:
    #     if connection[0] > max_vertex:
    #         max_vertex = connection[0]
    #     if connection[1] > max_vertex:
    #         max_vertex = connection[1]
    # # create vertice from 0 to max_vertex
    # vertices = []
    # for i in range(max_vertex+1):
    #     vertices.append(i)
    # # create graph and do dijkstra_distance() from staring vertex to end vertex
    # graph = Graph(V=vertices, E=connections, maxIn=maxIn, maxOut=maxOut)
    # print(ford_fulkerson(graph, origin, targets))