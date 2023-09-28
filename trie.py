#node data structure 
# I refere the week 11 lecture
class Node:
    def __init__(self) -> None:
        # terminal $ at index 0
        self.link = [None] * 27
        # data payload
        self.char = None
        self.max_freq = 0
        self.next_index = None

# the trie data structure
# I refere the week 11 lecture
class Trie:
    def __init__(self) -> None:
        self.root = Node()
        
    def search(self, key):
        # begin from root
        current = self.root
        return_str = ""
        # go through the key 1 by 1
        for char in key:
            # calculate index
            index = ord(char) - 97 + 1
            # if path exist
            if current.link[index] is not None:
                current = current.link[index]
                return_str += current.char
            # if path doesn't exist
            else:
                return None
        # go through the terminal $    
        # if path exist
        if current.link[0] is not None:
            while current.next_index != None:
                current = current.link[current.next_index]
                current_char = current.char
                return_str += current_char
        # if path doesn't exist
        else:
            while current.next_index != None:
                current = current.link[current.next_index]
                current_char = current.char
                return_str += current_char
        return return_str
            
    def insert_recur(self, key):
        current = self.root
        self.insert_recur_aux(current, key, 0)
        
    def insert_recur_aux(self, current, key, times):
        # base case
        if len(key) == times:
            # what happen when i go through all of my alpha in key
            # go through the terminal $ 
            # if path exist
            if current.link[0] is not None:
                current.link[0].max_freq += 1
                max_freq = current.link[0].max_freq
                if max_freq >= current.max_freq:
                    current.next_index = None
                current = current.link[0]
            # if path doesn't exist
            else:
                current.link[0] = Node()
                current.link[0].max_freq += 1
                max_freq = current.link[0].max_freq
                if max_freq >= current.max_freq:
                    current.next_index = None
                current = current.link[0]
            return max_freq
        # recur
        else:
            # calculate index
            index = ord(key[times]) - 97 + 1
            previous = current
            # if path exist
            if current.link[index] is not None:
                current = current.link[index]
            # if path doesn't exist
            else:
                current.link[index] = Node()
                current.link[index].char = key[times]
                current = current.link[index]
            times += 1
            max_freq = self.insert_recur_aux(current, key, times)
            if current.max_freq < max_freq:
                current.max_freq += 1
            if previous.max_freq < max_freq:
                previous.max_freq = max_freq
                previous.next_index = index
            if previous.max_freq == max_freq and previous.next_index is not None and previous.next_index >= index:
                previous.max_freq = max_freq
                previous.next_index = index
            return max_freq

class CatsTrie:
    def __init__(self, sentences) -> None:
        self.trie = Trie()
        for sentence in sentences:
            self.trie.insert_recur(sentence)
            
    def autoComplete(self, search_word):
        return_str = self.trie.search(search_word)
        return return_str
        
            

sentences = ['abc', 'abc', 'ab']

# Creating a CatsTrie object
mycattrie = CatsTrie(sentences)        
prompt = "a"
return_str = mycattrie.autoComplete("")
print(return_str)

# trie = Trie()
# trie.insert_recur("lolok")
# trie.insert_recur("lolok")
# trie.insert_recur("lolok")
# trie.insert_recur("lolok")
# trie.insert_recur("lolok")
# trie.insert_recur("lolok")
# trie.insert_recur("lolok")
# trie.insert_recur("los")
# trie.insert_recur("lssgit")
# trie.insert_recur("weii")
# trie.insert_recur("we")
# trie.insert_recur("we")
# trie.insert_recur("gii")
# trie.insert_recur("ceii")
# trie.insert_recur("seii")

# print(trie.search("we"))
    