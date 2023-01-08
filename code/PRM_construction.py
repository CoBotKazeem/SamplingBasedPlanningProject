# Probabilistic Road Map Construction
import numpy as np
import csv

#local function that will strip comment lines from the csv files
def decomment(csvfile):
    for row in csvfile:
        raw = row.split('#')[0].strip()
        if raw: yield row

#This function takes the start and goal coordinates and generates a grid of nps +1 nodes between them
def grid_create(start, goal, nps):
    space_x = abs((goal[0] - start[0]))/nps
    space_y = abs((goal[1] - start[1]))/nps
    #creates a zeroed list with the dimensions nps+1 x nps+1
    nodes = [[0] * (nps+1) for i in range(nps+1)]

    #fills in the dimensions of the nodes by adding equally spaced segments from the start
    for x in range (nps+1):
        for y in range (nps+1):
            nodes[x][y] = [(start[0] + x * space_x), (start[1] + y * space_y)]
    #Breaks down the 2D list into one long list
    flat_list = []
    for i in nodes:
        for j in i:
            flat_list.append(j)
    return (flat_list, round(space_x,3))

#This function takes a list of nodes and compares them to point obstacles with a defined radius
#nodes found within an obstacle are removed
def node_collision_detection (nodeslist, obstacles, goal, RoR):
    collisionpoints = []
    viablenodes = []
    for i in obstacles:
        for j in nodeslist:
            #calculates the distance from the node to the center of the obstacle
            distance = np.sqrt((i[0]-j[0])**2 + (i[1]-j[1])**2)
            # if the node is within the obstacle, it is removed from the nodeslist
            if distance <= i[2]/2 + RoR:
                #print(i, j, ' COLLISION')
                if j not in collisionpoints:
                    collisionpoints.append(j)
    #print(collisionpoints)
    for i in collisionpoints:
        nodeslist.remove(i)
    nodeindex=1
    #after the colliding nodes are removed, the remaining nodes are assigned an index number and the cost to go to the goal is calculated
    for i in nodeslist:
        nodectg = np.sqrt((i[0]-goal[0])**2 + (i[1]-goal[1])**2)
        nodeline = int(nodeindex), round(i[0],3), round(i[1],3), round(nodectg,3)
        viablenodes.append(nodeline)
        nodeindex = nodeindex +1
    return viablenodes

#This function will return a list of edges that do not pass through obstacles.
def create_edges (nodes, obstacles, sbn):
    edges = []
#    print(nodes)
    sbn = round(sbn,3) #space between nodes passed from the grid_create function
    #checks if there are direct neighbor nodes is +y direction, in +x direction, and diagonally in +x and +y
    for node in nodes:
        nbr_up = [round(node[1],3), round(node[2] + sbn,3)]
        nbr_right = [round(node[1]+ sbn,3), round(node[2],3)]
        nbr_diag = [round(node[1]+ sbn,3), round(node[2] + sbn,3)]
#        print(nbr_up, nbr_right, nbr_diag)
        #finds and adds the node indexes of the neighbor and node along with the distance between them
        for nbr in nodes:
            if nbr[1] == nbr_up[0] and nbr[2] == nbr_up[1]:
                cost = np.sqrt((node[1]-nbr[1])**2 + (node[2]-nbr[2])**2)
                line = (round(nbr[0],3), round(node[0],3), round(cost,3))
                edges.append(line)
            elif nbr[1] == nbr_right[0] and nbr[2] == nbr_right[1]:
                cost = np.sqrt((node[1]-nbr[1])**2 + (node[2]-nbr[2])**2)
                line = (round(nbr[0],3), round(node[0],3), round(cost,3))
                edges.append(line)
            elif nbr[1] == nbr_diag[0] and nbr[2] == nbr_diag[1]:
                cost = np.sqrt((node[1]-nbr[1])**2 + (node[2]-nbr[2])**2)
                line = (round(nbr[0],3), round(node[0],3), round(cost,3))
                edges.append(line)
    possible_collide = []
    for obstacle in obstacles:
        x = obstacle[0]
        y = obstacle[1]
        rad = obstacle[2]/2
        for line in edges:
            x1 = nodes[line[0]-1][1]
            y1 = nodes[line[0]-1][2]
            x2 = nodes[line[1]-1][1]
            y2 = nodes[line[1]-1][2]
            #perpendicular distance from full line passing through the segment points to the center of obstacle
            dist = (abs(((x2 - x1)*(y1 - y)) - (x1 - x)*(y2 - y1))) / np.sqrt(((x2 - x1)**2) + ((y2 -y1)**2))
            #since I removed all nodes within obstacles when generating the grid,
            # just have to check if edge is a chord across the obstacle
            if dist <= rad and line not in possible_collide:
                #vertical line segment
                if y1 == y2 and x1 < x < x2:
                    print(x1, y1, x2, y2,obstacle, 'COLLISION')
                    possible_collide.append(line)
                #horizontal line segment
                elif x1 == x2 and y1 < y < y2:
                    print(x1, y1, x2, y2, obstacle, 'COLLISION')
                    possible_collide.append(line)
                #diagonal line segment
                elif x1 < x < x2 and y1 < y < y2:
                    print(x1, y1, x2, y2, obstacle, 'COLLISION')
                    possible_collide.append(line)

#    print(possible_collide)
    for line in possible_collide:
        edges.remove(line)

    return edges

obstaclelist = []
# Set fileloc = '' if you just want to read and write csv files to py file location
fileloc = 'C:\\Python\\SamplingBasedPlanningProject\\results\\'
# Open the obstacles file, strip out commented lines with function, then append good lines to local list
with open(fileloc + 'obstacles.csv') as csvfile:
        reader = csv.reader(decomment(csvfile))
        for row in reader:
            obstaclelist.append(row)
#Converting obstaclelist elements into floats
for i in range(len(obstaclelist)):
    obstaclelist[i][0] = float(obstaclelist[i][0])
    obstaclelist[i][1] = float(obstaclelist[i][1])
    obstaclelist[i][2] = float(obstaclelist[i][2])

start = [-0.5, -0.5] # x and y coord of the start node
goal = [0.5, 0.5] # x and y coord of the goal node
nps = 10 # nodes per segment. Will make a nps+1 x nps+1 grid of nodes between start and goal
RoR = 0.0 # radius of robot. Added to collision detection. Zero treats robot as point.

#finally call the functions defined above to do the work
nodesgrid, spacing = grid_create(start, goal, nps)
viable_nodes = node_collision_detection (nodesgrid, obstaclelist, goal, RoR)
edges = create_edges(viable_nodes, obstaclelist, spacing)

#Then we save the lists to csv files in the same file location as the obstacles csv file that was imported above
with open(fileloc + 'nodes.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    for line in viable_nodes:
        writer.writerow(line)
with open(fileloc + 'edges.csv', 'w', newline='') as f:
    writer = csv.writer(f)
    for line in edges:
        writer.writerow(line)

#Here I start the AStar algorithm to search for the lowest cost path to the goal based off the nodes and edges created
edgeslist = edges
nodeslist = viable_nodes

# create empty lists that are used below
nodecostindex = []
past_cost = []
optctg = []
esttotcost = []
parentnode = []
closelist = []

# This loop creates an initial table of nodes and costs
# Uses 99.9 instead of infinity because I couldn't count that high
for i in range(len(nodeslist)):
    nodecostindex.append(nodeslist[i][0])
    optctg.append(nodeslist[i][3])
    parentnode.append('-')
    if i == 0:
        past_cost.append(0)
        esttotcost.append(float(nodeslist[i][3]))
    else:
        past_cost.append(99.9)
        esttotcost.append(99.9)

# The goal node is the last node listed in nodes.csv file
goal = nodeslist[-1][0]

# Start the openlist with the first node with its estimated total cost
openlist = [[nodeslist[0][0], esttotcost[0]]]

# Pretty much followed the psuedocode in CH 10.2.4.1 of MR
while (len(openlist) > 0):
    currentnode = openlist[0][0]
    openlist.remove(openlist[0])
    closelist.append(currentnode)
    if currentnode == goal:
        print('SUCCESS!')
        finalroute = [int(goal)]
        parentidx = -1

        # This while loop uses the difference between indexes from the end to find the path back to start
        # May have been neater if I had zipped my lists into one table? I couldn't figure it out.
        while parentnode[parentidx] != '-':
            finalroute.append(int(parentnode[parentidx]))
            parentdelta = int(finalroute[-2]) - int(finalroute[-1])
            parentidx = parentidx - parentdelta
        finalroute = sorted(finalroute)
        # print(finalroute)
        with open(fileloc + 'path.csv', 'w') as f:
            writer = csv.writer(f)
            writer.writerow(finalroute)

    # loop searches both first and second point of edge vector in case list wasn't sorted
    for i in range(len(edgeslist)):
        for j in range(2):

            # this flips the index so if currentnode found at 0 returns 1 value for neighbor and vice versa
            nbr = edgeslist[i][abs(j - 1)]
            nbrindex = nodecostindex.index(nbr)
            if edgeslist[i][j] == currentnode and nbr not in closelist:
                tent_past_cost = float(past_cost[nodecostindex.index(currentnode)]) + float(edgeslist[i][2])

                # If the neighbor cost is smaller than other searches, the tables are updated and openlist is sorted by lowest cost
                if tent_past_cost < past_cost[nbrindex]:
                    past_cost[nbrindex] = tent_past_cost
                    parentnode[nbrindex] = currentnode
                    esttotcost[nbrindex] = float(past_cost[nbrindex]) + float(optctg[nbrindex])
                    openlist.append([nbr, esttotcost[nbrindex]])
                    openlist.sort(key=lambda openlist: openlist[1])

# If the search never reached the goal then it fails
if goal not in closelist and parentnode[-1] == '-':
    print('Dead End')