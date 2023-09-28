##########################################################################################################
# 1. Fast Backups
##########################################################################################################
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
        discovered = []
        discovered.append(self.origin)
        # while length of discovered is bigger than 0, loop
        while len(discovered) > 0:
            # pop top value from discovered and get vertex
            u_id = discovered.pop(0)
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
                if v.discovered == False:
                    discovered.append(v_id)
                    v.discovered = True
                    v.previous = u_id
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
        # make path and discovered list and append origin to discovered
        path = []
        discovered = []
        discovered.append(self.origin)
        # while length of discovered is bigger than 0
        while len(discovered) > 0:
            # pop the top value and get vertex u by using u_id. and make u.visited true
            u_id = discovered.pop(0)
            u = self.graph.vertices[u_id]
            u.visited = True
            # loop for edges of u
            for edge in u.edges:
                v_id = edge.v
                v = self.graph.vertices[v_id]
                # if v.discovered is False, change v.discovered is true and add v to discovered, and v.previous is id of u
                if v.discovered == False:
                    discovered.append(v_id)
                    v.discovered = True
                    v.previous = u_id
        
        # set previous as previous of sink vertex and append sink to path
        vertices = self.graph.vertices
        previous = vertices[self.sink].previous
        path.append(self.sink)
        # while previous is not origin
        while previous != self.origin:
            # append previous to path and change previous with previous of previous vertex
            path.append(previous)
            previous = vertices[previous].previous
        # append origin to path
        path.append(self.origin)
        # reverse path
        path.reverse()
        
        # set infinity to residual_capacity
        residual_capacity = float("inf")
        # for loop for range of path length - 1
        for i in range(len(path) - 1):
            # get the vertex of the path which is positioning i in path
            path_vertex = vertices[path[i]]
            # if maxOut is smaller than residual_capacity, the residual_capacity become max_out
            max_out = path_vertex.maxOut
            if max_out < residual_capacity:
                residual_capacity = max_out
            # for loop for edges of path_vertex
            for edge in path_vertex.edges:
                # get the edge which is connecting to next path
                if edge.v == path[i+1]:
                    # if max_throughput is smaller than residual_capacity, residual_capacity become max_throughput
                    max_throughput = edge.w
                    if max_throughput < residual_capacity:
                        residual_capacity = max_throughput
            # if maxIn is smaller than residual_capacity, the residual_capacity become max_in
            max_in = vertices[path[i+1]].maxIn
            if max_in < residual_capacity:
                residual_capacity = max_in
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
        this_path = path.path
        vertices = self.graph.vertices
        # looping for range of length of this_path - 1
        for i in range(len(this_path) - 1):
            # get path_vertex with this_path[i]
            path_vertex = vertices[this_path[i]]
            # minus residual_capacity from maxOut
            path_vertex.maxOut -= residual_capacity
            # looping for edges of path_vertex
            for edge in path_vertex.edges:
                # get the edge which is connecting to next path
                if edge.v == this_path[i+1]:
                    # minus residaul_capacity from edge weight
                    edge.w -= residual_capacity
                    # if edge weight become 0, remove the edge from edges in path_vertex
                    if edge.w == 0:
                        path_vertex.edges.remove(edge)
            # minus residual_capacity from maxIN of the vertex of next path.
            vertices[this_path[i+1]].maxIn -= residual_capacity
            # if maxOut or maxIn is 0, then remove every edges in path_vertex
            if path_vertex.maxOut == 0 or path_vertex.maxIn == 0:
                path_vertex.edges.clear()
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

    
    # as long as there is an agumenting path
    while residual_network.has_AugmentingPath():
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

##########################################################################################################
# 2. catGPT
##########################################################################################################

#node data structure 
class Node:
    """
    I refer to the Malaysia week 11 Lecture Video explained by Mr.Lim Wern Han
    """
    def __init__(self) -> None:
        """
		Function description: This function initilizes Node by creating link, char, max_freq and next_idex

		Approach description: Each Node should have link, char, max_freq and next_index
		
		Input: this function does not have any inputs
		
		Output: This function does not return anything.
		
		Time complexity: O(1)
		Aux space complexity: O(1)
  
        """
        # terminal $ at index 0
        self.link = [None] * 27
        # data payload
        # char for this node
        self.char = None
        # max freqency of word which can be reached from this node
        self.max_freq = 0
        # index which next node containes
        self.next_index = None

# the trie data structure
class Trie:
    """
    I refer to the Malaysia week 11 Lecture Video explained by Mr.Lim Wern Han
    """
    def __init__(self) -> None:
        """
		Function description: This function initilizes Trie by creating root

		Approach description: Each Trie should have root
		
		Input: This function does not have any inputs
		
		Output: This function does not return anything.
		
		Time complexity: O(1)
		Aux space complexity: O(1)
  
        """
        # initialize root with empty Node
        self.root = Node()
        
    def search(self, key):
        """
		Function description: This function returns the search result of input

		Approach description: Start from root and check whether there is the node for key, after checking key, loop while current.next_index is not None
                            then add every char which went though to the return_str.
		
        Looping for key takes O(X) where X is the the length of the prompt. Looping while current.next_index is not None takes O(Y)
        where Y is the length of the most frequent sentence in sentences that begins with the prompt. Thus, in total there is O(X + Y) time complexity.
        
		Input: 
            key: a string with characters in the set of [a...z]. This string represents the incomplete sentence that is to be completed by the trie.

		
		Output: This function return return_str which contain of auto complete string result
		
		Time complexity: O(X + Y), where X is the the length of the prompt and Y is the length of the most frequent sentence in sentences that begins with the prompt.
		Aux space complexity: O(X + Y), where X is the the length of the prompt and Y is the length of the most frequent sentence in sentences that begins with the prompt.
  
        """
        # begin from root
        current = self.root
        # initialize return_str which contain of auto complete string result
        return_str = ""
        # go through the key 1 by 1
        for char in key:
            # calculate index
            index = ord(char) - 97 + 1
            # if path exist, change the current by using index and add current.char to return_str
            if current.link[index] is not None:
                current = current.link[index]
                return_str += current.char
            # if path doesn't exist, return None
            else:
                return None
        # go through the terminal $    
        # while current.next_index is not None, loop
        while current.next_index != None:
            # change current with current.next_index, and store current.char to return_str.
            current = current.link[current.next_index]
            current_char = current.char
            return_str += current_char
        # return the return
        return return_str
            
    def insert_recur(self, key):
        """
		Function description: This function start recursion to insert key to Trie.

		Approach description: set current as self.root and run insert_recur_aux()
  
        Since in recursion, it needs to go through every character in the key, so it takes O(M), M is the number of characters in the key.
        
		Input: 
            key: a string with characters in the set of [a...z]. 

		
		Output: This function does not return anything
		
		Time complexity: O(M), M is the number of characters in the key
  
        """
        # set self.root as current
        current = self.root
        # call insert_recur_aux with current and key and position 0
        self.insert_recur_aux(current, key, 0)
        
    def insert_recur_aux(self, current, key, times):
        """
		Function description: This function insert key in Trie.

		Approach description: Do recursion and find the place where there is no Node for the key and create new Node, then change next_index.
                            If reached base case, if there is no Node at the end create. Then add 1 to max_freq and compare with the previous max_freq,
                            if new max_freq is smaller then current.next_index become None.
        
        Since in recursion, it needs to go through every character in the key, so it takes O(M), M is the number of characters in the key.
        
		Input: 
            current: the Node where currently checking.
            key: a string with characters in the set of [a...z]. 
            times: the position of the key, where currently checking.

		
		Output: This function returns max_freq which has gotten in the base case
		
		Time complexity: O(M), M is the number of characters in the key
  
        """
        # base case
        # if len(key) is same as times, this means when when this function went through every character recursively
        if len(key) == times:
            # go through the terminal $ 
            # if path exist
            # if current.link has "$"
            if current.link[0] is not None:
                # add 1 to the max_freq of the $ in current.link and get the max_freq
                current.link[0].max_freq += 1
                max_freq = current.link[0].max_freq
                # if max_freq is bigger than max_freq of current, this means freqency of this key is bigger than the word which contain this key.
                if max_freq >= current.max_freq:
                    # current.next_index is None
                    current.next_index = None
                # change the current to the current.link[0]
                current = current.link[0]
            # if path doesn't exist
            else:
                # create "$" with empty Node and add 1 to max_freq
                current.link[0] = Node()
                current.link[0].max_freq += 1
                max_freq = current.link[0].max_freq
                # if max_freq is bigger than max_freq of current, this means freqency of this key is bigger than the word which contain this key.
                if max_freq >= current.max_freq:
                    # current.next_index is None
                    current.next_index = None
                # change the current to the current.link[0]
                current = current.link[0]
            return max_freq
        # recur
        else:
            # calculate index
            index = ord(key[times]) - 97 + 1
            # set previsou with current
            previous = current
            # if path exist
            if current.link[index] is not None:
                # change current with current.link[index]
                current = current.link[index]
            # if path doesn't exist
            else:
                # create Node in current.link[index] and store char with key[times], and change current with current.link[index]
                current.link[index] = Node()
                current.link[index].char = key[times]
                current = current.link[index]
            # increease times by 1, means change the position of checking key
            times += 1
            # call insert_recur_aux and get max_freq from that.
            max_freq = self.insert_recur_aux(current, key, times)
            # if current.max_freq is smaller than max_freq of the end of key, add 1 to current.max_freq
            if current.max_freq < max_freq:
                current.max_freq += 1
            # if previous.max_freq is less than max_freq of the end of key
            # if previous.max_freq is same as max_freq of the end of key, and previous.next_indes is not None and previous.next_index is bigger than 
            # index, this means if the max_freq is same and next_index of previous is bigger alphabet than index.
            if previous.max_freq < max_freq or (previous.max_freq == max_freq and previous.next_index is not None and previous.next_index > index):
                # change previous.max_freq with max_freq of the end of key
                previous.max_freq = max_freq
                # previous.next_index is index
                previous.next_index = index
            # return max_freq
            return max_freq

class CatsTrie:
    def __init__(self, sentences) -> None:
        """
	    Function description: This function initilizes CatTrie by creating trie and inserting every sentence
	    Approach description: Initializing trie and do looping for sentences and insert sentence using insert_recur
    
        insert_recur takes O(M), where M is the number of characters in the key. Looping for sentences takes O(N), where N is the number of sentence in sentences.
        Thus, this functions takes O(NM), where N is the number of sentence in sentences and M is the number of characters in the longest sentence.
        
	    Input: 
            sentences: a list of strings 
    
	    Output: This function does not return anything.
    
	    Time complexity: O(NM), where N is the number of sentence in sentences and M is the number of characters in the longest sentence.
	    Aux space complexity: O(NM), where N is the number of sentence in sentences and M is the number of characters in the longest sentence.

        """
        #Initialize Trie
        self.trie = Trie()
        # loop for sentences and insert sentence with insert_recur in Trie.
        for sentence in sentences:
            self.trie.insert_recur(sentence)
            
    def autoComplete(self, search_word):
        """
	    Function description: This function return the result of auto complete for search word
	    Approach description: Run search function in trie by using search_word, and return it.
        
	    Input: 
            search_word: A string with characters in the set of [a...z]. This string represents the incomplete sentence that is to be completed by the trie.
    
	    Output: This function returns a string that represents the completed sentence from the prompt.
    
	    Time complexity: O(X + Y), where X is the the length of the prompt and Y is the length of the most frequent sentence in sentences that begins with the prompt.
	    Aux space complexity: O(X + Y), where X is the the length of the prompt and Y is the length of the most frequent sentence in sentences that begins with the prompt.

        """
        # run the search function in trie by using search_word
        return_str = self.trie.search(search_word)
        # return a string that represents the completed sentence from the prompt.
        return return_str
