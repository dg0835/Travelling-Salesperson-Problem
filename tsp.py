import numpy as np
import itertools 
import math
from operator import attrgetter
import random

def calculateDistanceBetweenNodes(a, b): #Uses Pythagorean theorem to calculate the distance between 2 nodes.
    xDistance = a.getX() - b.getX()
    xDistance = xDistance**2
    yDistance = a.getY() - b.getY()
    yDistance = yDistance**2
    totalDistance = np.sqrt(xDistance + yDistance)
    return totalDistance    

class Node:
    def __init__(self, label, x, y, pointsTo, idnum):
        self.label = label
        self.x = x
        self.y = y
        self.pointsTo = []
        
    def getLabel(self):
        return self.label
    
    def getX(self):
        return self.x
    
    def getY(self):
        return self.y
    
    def getPointsTo(self):
        return self.pointsTo
    
    def setPointsTo(self, nodes):
        for i in nodes:
            self.pointsTo.append(i)
            
    def appendToPointsTo(self, dest):
        self.pointsTo.append(dest)
            
    def getDegree(self):
        degree = len(self.pointsTo)
        return degree
        
    def setID(self, idnum):
        self.idnum = idnum
        
    def getID(self):
        return self.idnum
        
    def addEdges(self, pointsTo):
        for i in self.pointsTo:
            label = str(self.label) + "-" + str(i.label)
            edge = Edge(self, i, calculateDistanceBetweenNodes(self, i), self.label)
            edge.setLabel(label)
            edges.append(edge)
            
    def clearConnections(self):
        self.pointsTo.clear()
        
    def createConnection(d): #Creates an edge using a source and destination
        edge = Edge(self, d, calculateDistanceBetweenNodes(self, d), str(self.label) + "-" + str(d.label))
        
    
class Edge:
    def __init__(self, start, dest, weight, label):
        self.start = start
        self.dest = dest
        self.weight = weight
        self.label = str(start.label) + "-" + str(dest.label)
        
    def calculateWeight(self, start, dest):
        calculateDistanceBetweenNodes(start, dest)
        return weight
        
    def getWeight(self):
        return self.weight
       
    def setLabel(self, label):
        self.label = label
    
    def getLabel(self):
        return self.label
    
    def getStart(self):
        return self.start
    
    def getDest(self):
        return self.dest

    def setStart(self, n):
        self.start = n
        
    def setDest(self, n):
        self.dest = n
        

def createEdge(s, d):
    e = Edge(s, d, calculateDistanceBetweenNodes(s, d), s.getLabel() + "-" + d.getLabel())
    return e


def checkNumeric(message): #Validity check to check for a numeric input (invalid means the input must be done again)
    while True:
        answer = input(message)
        if answer.isnumeric():
            return answer
        else:
            print('Invalid input, please try again.')
            
def checkDecimal(message): #Checks for validity by handling for a ValueError exception if it occurs
    while True:
        try:
            answer = input(message)
            float(answer)
            return answer    
        except ValueError:
            print('Invalid input, please try again.')
    
    
def labelNodeList(route): #Creates a user-friendly list of nodes, which displays the node labels instead of a more unintelligible output, showing the node positions in memory
    labelledList = []
    for i in route:
        label = i.getLabel()
        labelledList.append(label)
        
    return labelledList


class ShortestRoute:
    def __init__(self, distance, route):
        self.distance = distance
        self.route = route
        
    def getDistance(self):
        return self.distance
    
    def getRoute(self):
        return self.route
    
    def setDistance(self, n):
        self.distance = n
    
    def setRoute(self, route):
        self.route = route
            
        
    def displayShortestRoute(self):
        bestRoute = self.getRoute()
        bestRoute = labelNodeList(bestRoute)
        print("\nThe shortest route for brute force algorithm is: " , bestRoute)
        print("The shortest route length is " , self.getDistance() , "\n")


numberOfNodes = checkNumeric('How many nodes would you like to add? Please enter a positive integer as your input: ')

def generateRandomNodes(numberOfNodes):
    nodes = []
    for i in range(0, numberOfNodes): #For each node there will be
        nodex = random.randrange(1000) #Give it a random X coordinate between 0 and 999
        nodey = random.randrange(1000) #Also a random Y coordinate with the same constraints
        nodelabel = "X" + str(nodex) + "Y" + str(nodey) #The node label will correspond to its X and Y coordinates
        n = Node(nodelabel, nodex, nodey, None, None)
        nodes.append(n)
    return nodes


def initialiseNodes(numberOfNodes): #Nodes are created here by the user inputting their coordinates and labels.
    nodes = []
    for i in range(int(numberOfNodes)):
        nodelabel = input('Enter the node label: ')
        nodex = checkDecimal('Enter x coordinate for node ' + str(i+1) + ': ')
        nodex = float(nodex)
        nodey = checkDecimal('Enter y coordinate for node ' + str(i+1) + ': ')
        nodey = float(nodey)
        n = Node(nodelabel, nodex, nodey, None, None)
        nodes.append(n)
    return nodes

NodeGenerationChoiceValid = False

while NodeGenerationChoiceValid == False:
    NodeGenerationChoice = input('Input 1 if you would like the nodes to be randomly generated, or input 2 to enter the label and coordinates of each node yourself.')
    if NodeGenerationChoice == "1":
        nodes = generateRandomNodes(int(numberOfNodes))
        NodeGenerationChoiceValid = True
        
    elif NodeGenerationChoice == "2":
        nodes = initialiseNodes(int(numberOfNodes))
        NodeGenerationChoiceValid = True

    else:
        print("Invalid input - Please enter the number 1 or 2.")

print("The nodes are: ")
for n in nodes:
    print(n.getLabel())
        

edges = []

sr = ShortestRoute(math.inf, None)

nodeLabels = [] 

def pointToAllOtherNodes():#This is applied to all nodes to form a completed graph (where you can go from one node to any other node)
    for i in nodes:
        i.setPointsTo(nodes)
        for j in i.getPointsTo():
            if i == j:
                i.getPointsTo().remove(j)

def settingEdgesAndLabelledNodesList(): #Initialises 2 lists
    for i in nodes:
        nodeLabels.append(i.getLabel())
        i.addEdges(i.getPointsTo())


def returnToStartDistance(currentNode, startingNode, totalDistance): #Finds the distance to return to the starting node from the current one.
    distance = calculateRouteDistance(currentNode, startingNode)
    totalDistance += distance
    return totalDistance

def orderEdges(edges, start): #Orders the edges so the first edge is the one from the starting node
    ordered = []
    target = start.getLabel()
    for i in range(0, len(edges)):
        for e in edges:
            if e not in ordered:
                if e.getStart().getLabel() == target:
                    ordered.append(e)
                    target = e.getDest().getLabel()
                elif e.getDest().getLabel() == target:
                    a = e.getStart()
                    b = e.getDest()
                    e.setStart(b)
                    e.setDest(a)
                    ordered.append(e)
                    target = e.getDest().getLabel()
    #for i in ordered:
        #print("ordered list : " , i.getLabel())
    return ordered


def calculateRouteDistance(route): #Finds total euclidean distance of a route passed as an argument by finding the sum of weights of all edges
    totalDistance = 0
    for i in range(len(route)):
        currentNode = i
        if currentNode == len(route)-1:
            nextNode = 0 #For the final iteration, ensure that the final nextNode is the node that you started at, so that TSP is valid.
        else:
            nextNode = i+1 #For any other iteration, the next node will be the one after the current node in the list.
        distance = calculateDistanceBetweenNodes(route[currentNode], route[nextNode]) 
        totalDistance = totalDistance + distance
    if totalDistance < sr.getDistance():
        sr.setDistance(totalDistance)
        sr.setRoute(route)
    #print("Total distance of this route: " , labelNodeList(route) , " is " , totalDistance)
    
def findFinalEdge(nodes): #Adds the final edge to return to the starting node. Does this by finding the two nodes which have a degree of 1, which is always the current node and the starting node.
    finalEdge = []
    for i in nodes:
        #print("degree of " , i.getLabel() , "is " , i.getDegree())
        if i.getDegree() == 1:
            finalEdge.append(i)
            
    #edgelabel = finalEdge[0].getLabel(), " - ", str(finalEdge[1].getLabel())
    start = finalEdge[0]
    dest = finalEdge[1]
    
    #print(start.getLabel())
    #print(dest.getLabel())
            
    e = createEdge(start, dest)
   # print("label of final edge " , e.getLabel())
    return e
    
def bruteForce(nodes):
    permutations = list(itertools.permutations(nodes, len(nodes))) #Finds all possible orders of elements in the list once. Necessary for the brute force algorithm.
    startingNode = nodes[0]
    counter = 0
    for i in range(np.math.factorial(len(nodes))): #The number of possible paths is equal to the factorial of the number of nodes #The number of possible paths is equal to the factorial of the number of nodes
        currentPermutation = permutations[counter]
        counter += 1
        print(labelNodeList(currentPermutation))
        if currentPermutation[0].getLabel() == startingNode.getLabel(): 
            calculateRouteDistance(currentPermutation)
    return sr.displayShortestRoute()
        
def nearestNeighbour(nodes, whatToReturn):
    nodelist = []
    unvisitedNodes = []
    for i in nodes:
        unvisitedNodes.append(i)
    currentNode = unvisitedNodes[0]
    startingNode = unvisitedNodes.pop(0)
    nodelist.append(startingNode)
    totalDistance = 0
    for i in range(len(nodes)-1):
        nnDistance = math.inf #The distance to the nearest neighbour. Initialised as infinity each time we look for the nearest neighbour to guarantee that there will be a nearest neighbour if there's at leeat one unvisited node
        nextNode = None
        for j in unvisitedNodes: #This for loop attempts to find the nearest neighbour to the current node. The nearest neighbour found will be stored in the variable nextNode
            distance = calculateDistanceBetweenNodes(currentNode, j)
            print("Distance between " , currentNode.getLabel(), " and " , j.getLabel() , " is " , distance)
            if distance < nnDistance:
                nnDistance = distance
                nextNode = j
        totalDistance += nnDistance
        currentNode = nextNode
        nodelist.append(currentNode)
        #print("The current node is now " , currentNode.getLabel() , "as this is the nearest neighbour")
        #print(labelNodeList(unvisitedNodes))
        #print("current node- " , currentNode.getLabel())
        unvisitedNodes.remove(currentNode)
        #print("univisted nodes " , unvisitedNodes)
    #print("currentNode is " , currentNode.getLabel())
    #print("startingNode is " , startingNode.getLabel())
    finalEdgeDistance = calculateDistanceBetweenNodes(currentNode, startingNode)
    #print("Distance between " , currentNode.getLabel(), " and " , startingNode.getLabel() , " is " , finalEdgeDistance)
    totalDistance += finalEdgeDistance
    nodelist.append(startingNode)
    if whatToReturn == "routeAndDistance":
        print("The route found by the Nearest Neighbour algorithm is :" , labelNodeList(nodelist)," and has a distance of: " , totalDistance)
        return(labelNodeList(nodelist), totalDistance)
    elif whatToReturn == "distance":
        print("Distance for the route: " , labelNodeList(nodelist)), "is: " , totalDistance
        return totalDistance
    elif whatToReturn == "route":
        return labelNodeList(nodelist)
        
def getWeightsList(edges): #Return a list of all of the weights of each edge
    weights = []
    for i in edges:
        weights.append(i.getWeight())
    return weights

def sortEdges(edges): #Sorts the edges list in ascending order of weight
    unsortedEdges = edges
    sortedEdges = []
    weights = getWeightsList(edges)
    for i in range(len(unsortedEdges)):
        minIndex = weights.index(min(weights)) #find index of smallest weight
        sortedEdges.append(unsortedEdges[minIndex])#append smallest weighted edge to sortedEdges list
        del unsortedEdges[minIndex]#Remove the appended edge from unsortedEdges and weights to avoid duplicate edges
        del weights[minIndex]
    return sortedEdges

def setNodeIDs(nodes): #IDs ensure that cycles are not created.
    counter = 1
    for i in nodes:
        i.setID(counter)
        counter += 1

def isCycle(i): #Check for cycles. If there will be one, then reject the edge (for the route found by the greedy algorithm)
    if i.getStart().getID() != i.getDest().getID():
        return False
    else:
        return True
    
def isLargeDegree(i): #Check for degrees that are 2 or above. If this occurs, reject the eddge for the greedy algorithm.
    if i.getStart().getDegree() < 2 and i.getDest().getDegree() < 2:
        return False
    else:
        return True
    
def changeID(i, nodes): #Make the numeric ID of both nodes, connected by the edge i, equal so that cycles can continue to be avoided 
    originalStartID = i.getStart().getID()
    originalDestID = i.getDest().getID()
    minID = min([originalDestID, originalStartID])
    #for q in nodes:
        #print("ID of " , q.getLabel() , "is " , q.getID())
    
    for n in nodes:
        if n.getID() == originalStartID or n.getID() == originalDestID:
            #print("Change ID of " , n.getLabel() ,  "to " , minID)
            n.setID(minID)
        
        

def greedyAlgorithm(nodes, edges):
    startingNode = nodes[0]
    route = []
    readableRoute = []
    routesum = 0
    #print(edges)
    #Select the smallest weighted edge from the list of edges, append it to a new list and delete from old list 
    sortedEdges = sortEdges(edges)
    setNodeIDs(nodes)
    for q in sortedEdges:
        print(q.getLabel(), q.getWeight())
    
    for n in nodes:
        n.clearConnections()
    for i in sortedEdges:
        #print(i.getStart().getLabel() , " ID " , i.getStart().getID())
        #print(i.getDest().getLabel() , " ID " , i.getDest().getID())
        if not isCycle(i) and not isLargeDegree(i): #Checks that no cycles are formed and the degree of no nodes increases above 2.
            route.append(i)
            i.getStart().pointsTo.append(i.getDest())
            i.getDest().pointsTo.append(i.getStart())
            changeID(i, nodes)
            
            
    for j in route:
        routesum += j.getWeight()
        readableRoute.append(j.getLabel())
        #print("source of edge " , j.getLabel() , " is " , j.getStart().getLabel())
        #print("dest of edge " , j.getLabel() , " is " , j.getDest().getLabel())
    
    lastEdge = findFinalEdge(nodes)
    routesum += lastEdge.getWeight()
    route.append(lastEdge)
    o = orderEdges(route, startingNode)
    print("The route calculated by the greedy algorithm is: " , labelNodeList(o) , " with a total distance of " , routesum)
    

    return "The route calculated by the greedy algorithm is: " , labelNodeList(o) , " with a total distance of " , routesum

def rearrangeList(startingNodeIndex, nodes): #Rearrange a list of nodes to allow the starting node to change, but in doing so, keep the shape of the route the same
    rearrangedList = []
    numberOfNodes = len(nodes)
    for i in range(0, numberOfNodes):
        if startingNodeIndex >= numberOfNodes:
            startingNodeIndex = 0
        rearrangedList.append(nodes[startingNodeIndex])
        startingNodeIndex += 1
    return rearrangedList
    
    
def repeatedNearestNeighbour(nodes):
    startingPoint = nodes[0].getLabel()
    shortestDistance = math.inf
    shortestRoute = None
    numberOfNodes = len(nodes)
    for i in range(0, numberOfNodes): #Apply the Nearest Neighbour algorithm, where each node in the graph is used as a starting point.
        rearranged = rearrangeList(i, nodes)
        #print(labelNodeList(rearranged))
        distance = nearestNeighbour(rearranged, "routeAndDistance")
        route = distance[0]
        length = distance[1]
        #print(distance[0] , ": " , distance[1])
        if length < shortestDistance:
            shortestDistance = length
            shortestRoute = route
    print("Shortest route found by the repeated nearest neighbour is: " , shortestRoute)
    #shortestRoute must be rearranged such that the starting point is the same as it would be originally.
    startingPointIndex = shortestRoute.index(startingPoint)
    #print(startingPointIndex)
    finalRearrangedRoute = rearrangeList(startingPointIndex, shortestRoute)
    return finalRearrangedRoute

def randomChoice(nodes): #Randomly choose a number within a range
    randomIndex = random.randrange(len(nodes))
    return randomIndex 

#def printLabelledList(l, listname):
    #for i in range(0, len(l)):
        #print("element " , i, " of " , listname, ": ", l[i].getLabel())

def findNearestNode(source, unselectedNodes, shortestDistance, nearestNode, sourceNode):
    for j in unselectedNodes:
        d = calculateDistanceBetweenNodes(source, j)
        #print("distance between nodes " , source.getLabel() , " and " , j.getLabel() , " = " , d)
        if d < shortestDistance:
            nearestNode = j
            shortestDistance = d
            sourceNode = source
    
    return (nearestNode, shortestDistance, sourceNode)
    
def primsAlgorithm(nodes):
    unselectedNodes = [] #Nodes which haven't been inserted yet
    for i in nodes: 
        unselectedNodes.append(i)
    for n in nodes:
        n.clearConnections()
        
    edges.clear()
    calculatedRoute = []
    calculatedRoute.append(unselectedNodes[0])
    del unselectedNodes[0]
    
    while len(unselectedNodes) > 0:
        shortestDistance = math.inf 
        nearestNode = None
        sourceNode = None
        finalShortestDistance = math.inf
        finalSource = None
        finalNearest = None
        #printLabelledList(calculatedRoute, "calculated route")
        #printLabelledList(unselectedNodes, "unselected nodes")
        for i in calculatedRoute:
            #print("i is : " , i.getLabel())
            if i.getDegree() < 2: #Degree must not exceed 2 if a tour is to be created, where the features abide by the rules of TSP
                insertion = findNearestNode(i, unselectedNodes, shortestDistance, nearestNode, sourceNode)
                
                nearestNode = insertion[0]
                shortestDistance = insertion[1]
                sourceNode = insertion[2]
                
                
        if shortestDistance < finalShortestDistance:
            finalShortestDistance = shortestDistance
            finalSource = sourceNode
            finalNearest = nearestNode
            
        
        #print("final nearest " , finalNearest.getLabel())
        unselectedNodes.remove(finalNearest)
        calculatedRoute.append(finalNearest)
        
        finalSource.appendToPointsTo(finalNearest)
        finalNearest.appendToPointsTo(finalSource)
        
        
        #finalSource.addEdges(finalNearest)
        finalNearest.addEdges(finalSource)
        
    lastEdge = findFinalEdge(nodes)
    edges.append(lastEdge)
    o = orderEdges(edges, nodes[0])
        

    print("The calculated route for prim's algorithm is " , labelNodeList(o))
        
pointToAllOtherNodes()
settingEdgesAndLabelledNodesList()

def selectAlgorithm():
    algorithmChoice = ""
    while algorithmChoice != "exit":
        algorithmChoice = input("Select the algorithm you would like to use. Input: 1 for brute force, 2 for nearest neighbour, 3 for greedy algorithm, 4 for repeated nearest neighbour, 5 for Prim's algorithm, or type in exit to stop the program.")
        if algorithmChoice == "1":
            bruteForce(nodes)
        elif algorithmChoice == "2":
            print(nearestNeighbour(nodes, "routeAndDistance"))
        elif algorithmChoice == "3":
            greedyAlgorithm(nodes, edges)
        elif algorithmChoice == "4":
            print(repeatedNearestNeighbour(nodes))
        elif algorithmChoice == "5":
            print(primsAlgorithm(nodes))
        elif algorithmChoice == "exit":
            print("The program will stop running now.")
        else:
            print("Invalid input, please try again")

selectAlgorithm()

#bruteForce(nodes)
#print(nearestNeighbour(nodes, "routeAndDistance"))
#greedyAlgorithm(nodes, edges)
#print(edges)

#print(repeatedNearestNeighbour(nodes))

#print(primsAlgorithm(nodes))
