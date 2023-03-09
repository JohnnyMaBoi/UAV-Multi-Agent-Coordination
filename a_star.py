class Astar:
    """
    This class implements the A* algorithm to find paths on a given map

    Attributes:
        map (Map) : the environment for A*
        start (Tuple) : the starting coordinates of the drone
        target (Tuple) : the desired end coordinates of the drone
        nodes_found (List) : A list of nodes discovered by A*
        nodes_checked (List) : A list of nodes explored by A*
    Methods:
        __init__ (map, start, target): initializes the Astar class
        calculate_score(parent,child,target) : Calculates the heuristic value of a node
        pos_to_node(pos) : Finds the closest node to the given position
        a_star_search : implements the A* algorithm
    """
    def __init__(self, map, start, target):
        self.map = map
        self.start = start
        self.target = target
        self.nodes_found = []
        self.nodes_checked = []

    def calculate_score(self,parent,child,target):
        """
        Calculates the heuristic value of a node
        Args:
            Parent(Node):The previous node being explored
            Child(Node):The node being scored
            Target(tuple): The tuple containing the x and y position of the target
        Returns:
            Score (Double):The heuristic value of the node
        """
        # multi-agent note: implement score adjustment for idx in node.intersections
        _current_pos=child.coords
        _target_pos=self.target
        _dist_to_target=((_current_pos[0]-_target_pos[0])**2+(_current_pos[1]-_target_pos[1])**2)**0.5
        score=_dist_to_target+parent.heuristic+self.map.drone_dim

        return score

    def pos_to_node(self, pos):
        """
        Finds the closest node to the given position (localization)
        Args:
            pos(tuple): The tuple containing an x and y position in the map frame
            map(Map): the map of nodes for the position coords
        Returns:
            closest_node (Node):The node closest to the position
        """
        node_x = int(pos[0]//self.map.drone_dim)
        node_y = int(pos[1]//self.map.drone_dim)
        closest_node = self.map.array[node_y][node_x]
        return closest_node

    def a_star_search(self):
        """
        Implements the A* algorithm
        Args:
            None
        Returns:
            trajectory (List):The list of tuples defining the drone path
        """

        # creating a property that tells you where path has been
        #Starts A* with the drone starting position
        self.nodes_found.append(self.pos_to_node(self.start))

        while True:
            #If there are no nodes left to search, end the function
            if len(self.nodes_found) == 0:
                print("Tough luck, we couldn't find a solution")
                break
            
            #Find the next node to check (it has the lowest f_cost)
            self.nodes_found = sorted(self.nodes_found, key=lambda n: n.f_cost)
            node_to_check = self.nodes_found.pop(0)
            self.nodes_checked.append(node_to_check)

            #TARGET NODE FOUND! after check, enter into path assembly.
            if node_to_check == self.pos_to_node(self.target):
                #Creates a node path for the drone to follow, starting at ending point
                path = [node_to_check]
                node = node_to_check.parent
                while True:
                    # inserting the parent node before 1st node in list to form the path backwards
                    path.insert(0, node)
                    # once you reach the starting node, you're done making the path! 
                    if node == self.pos_to_node(self.start):
                        break
                    node = node.parent
                #Convert the path in nodes to the path in coordinates
                trajectory = []
                for traj_node in path:
                   trajectory.append(traj_node.coords)
                return trajectory
            
            #Checks nodes around the node being explored
            new_nodes = self.map.get_neighbors(node_to_check)
            for node in new_nodes:
                #Score each node
                node.set_heuristic(self.calculate_score(node_to_check,node,self.target))
                #Add the node if it hasn't been seen before
                if node not in self.nodes_found and node not in self.nodes_checked:
                    node.parent=node_to_check
                    self.nodes_found.append(node)
                #If the node has been checked before but has a different parent, use the parent with a lower score
                elif node in self.nodes_found and node.parent != node_to_check:
                    old_score = node.heuristic
                    old_parent = node.parent
                    node.set_parent(node_to_check)
                    new_score = node.heuristic
                    if old_score < new_score:
                        node.set_parent(old_parent)
                    else:
                        self.nodes_found.append(node)