import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import matplotlib.lines as lines
import os


create_video = True

plt.rcParams['animation.ffmpeg_path'] = os.path.join(os.getcwd(), 'ffmpeg.exe')

total_vertices = -1
start_vertex = -1
goal_vertex = -1
edges = {}
coordinates = {}
metadata = dict(title="Dijkstra Animation")
writer = FFMpegWriter(fps=30, metadata=metadata)
fig, axis = plt.subplots(2, 3)

def readInputFile():
    input_data_file = open("input.txt", "r")
    input_data_array = input_data_file.read().split('\n')
    input_data_file.close()
    
    global total_vertices
    total_vertices = int(input_data_array.pop(0))
    
    global start_vertex
    start_vertex = int(input_data_array.pop(0))
    
    global goal_vertex
    goal_vertex = int(input_data_array.pop(0))
    
    getEdges(input_data_array)
    
def getEdges(input_edges):
    for tuple in input_edges: 
        edge_values = tuple.split(' ')
        
        if len(edge_values) == 3:
            edge_tail = int(edge_values[0])
            edge_head = int(edge_values[1])
            edge_weight = float(edge_values[2])
            
            if edge_tail in edges:
                edges[edge_tail].append((edge_head, edge_weight))
            
            else:
                edges[edge_tail] = [(edge_head, edge_weight)]

def readCoordinateFile():
    input_coordinates_file = open("coords.txt", "r")
    input_coordinates = input_coordinates_file.read().split('\n')
    input_coordinates_file.close()

    for i in range(len(input_coordinates)):
        x_y_values = input_coordinates[i].split(' ')

        if len(x_y_values) == 2:
            coordinates[i+1] = (float(x_y_values[0]), float(x_y_values[1]))

def calculateHeuristic(goal_vertex):
    heuristic = {}
    goal_vertex= coordinates[goal_vertex]
    
    for vertex, coordinate in coordinates.items():
        heuristic[vertex] = pow(pow((goal_vertex[0]-coordinate[0]), 2) + pow((goal_vertex[1]-coordinate[1]), 2), 1/2)
    
    return heuristic

def a_star(heuristic):
    if create_video:
        graph = createGraph()
        axis = plotGraph(graph)
    
    total_weight_1 = float('inf')
    unvisited_vertices_1 = {}
    unvisited_heuristics_1 = {}
    previous_vertices_1 = {}

    total_weight_2 = float('inf')
    unvisited_vertices_2 = {}
    unvisited_heuristics_2 = {}
    previous_vertices_2 = {}

    total_weight_3 = float('inf')
    unvisited_vertices_3 = {}
    unvisited_heuristics_3 = {}
    previous_vertices_3 = {}

    total_weight_4 = float('inf')
    unvisited_vertices_4 = {}
    unvisited_heuristics_4 = {}
    previous_vertices_4 = {}

    total_weight_5 = float('inf')
    unvisited_vertices_5 = {}
    unvisited_heuristics_5 = {}
    previous_vertices_5 = {}

    total_weight_6 = float('inf')
    unvisited_vertices_6 = {}
    unvisited_heuristics_6 = {}
    previous_vertices_6 = {}
    
    for vertex in list(coordinates.keys()):
        previous_vertices_1[vertex]=[]
        previous_vertices_2[vertex]=[]
        previous_vertices_3[vertex]=[]
        previous_vertices_4[vertex]=[]
        previous_vertices_5[vertex]=[]
        previous_vertices_6[vertex]=[]
        
        if vertex == start_vertex:
            unvisited_vertices_1[vertex] = 0
            unvisited_vertices_2[vertex] = 0
            unvisited_vertices_3[vertex] = 0
            unvisited_vertices_4[vertex] = 0
            unvisited_vertices_5[vertex] = 0
            unvisited_vertices_6[vertex] = 0
            unvisited_heuristics_1[vertex] = 0
            unvisited_heuristics_2[vertex] = 0
            unvisited_heuristics_3[vertex] = 0
            unvisited_heuristics_4[vertex] = 0
            unvisited_heuristics_5[vertex] = 0
            unvisited_heuristics_6[vertex] = 0
        
        else:
            unvisited_vertices_1[vertex] = float('inf')
            unvisited_vertices_2[vertex] = float('inf')
            unvisited_vertices_3[vertex] = float('inf')
            unvisited_vertices_4[vertex] = float('inf')
            unvisited_vertices_5[vertex] = float('inf')
            unvisited_vertices_6[vertex] = float('inf')
            unvisited_heuristics_1[vertex] = float('inf')
            unvisited_heuristics_2[vertex] = float('inf')
            unvisited_heuristics_3[vertex] = float('inf')
            unvisited_heuristics_4[vertex] = float('inf')
            unvisited_heuristics_5[vertex] = float('inf')
            unvisited_heuristics_6[vertex] = float('inf')

    path_1, weights_1, done1 = None, None, False
    path_2, weights_2, done2 = None, None, False
    path_3, weights_3, done3 = None, None, False
    path_4, weights_4, done4 = None, None, False
    path_5, weights_5, done5 = None, None, False
    path_6, weights_6, done6 = None, None, False
    
    i = 0
    while len(unvisited_vertices_1) > 0 or len(unvisited_vertices_2) > 0 or len(unvisited_vertices_3) > 0 or len(unvisited_vertices_4) > 0 or len(unvisited_vertices_5) > 0 or len(unvisited_vertices_6) > 0:

        vertex_heuristics_1 = list(unvisited_heuristics_1.values())
        vertex_heuristics_2 = list(unvisited_heuristics_2.values())
        vertex_heuristics_3 = list(unvisited_heuristics_3.values())
        vertex_heuristics_4 = list(unvisited_heuristics_4.values())
        vertex_heuristics_5 = list(unvisited_heuristics_5.values())
        vertex_heuristics_6 = list(unvisited_heuristics_6.values())
        
        current_heuristics_1 = min(vertex_heuristics_1)
        current_heuristics_2 = min(vertex_heuristics_2)
        current_heuristics_3 = min(vertex_heuristics_3)
        current_heuristics_4 = min(vertex_heuristics_4)
        current_heuristics_5 = min(vertex_heuristics_5)
        current_heuristics_6 = min(vertex_heuristics_6)

        current_vertex_1 = [i for i in unvisited_heuristics_1 if unvisited_heuristics_1[i] == current_heuristics_1][0]
        current_vertex_2 = [i for i in unvisited_heuristics_2 if unvisited_heuristics_2[i] == current_heuristics_2][0]
        current_vertex_3 = [i for i in unvisited_heuristics_3 if unvisited_heuristics_3[i] == current_heuristics_3][0]
        current_vertex_4 = [i for i in unvisited_heuristics_4 if unvisited_heuristics_4[i] == current_heuristics_4][0]
        current_vertex_5 = [i for i in unvisited_heuristics_5 if unvisited_heuristics_5[i] == current_heuristics_5][0]
        current_vertex_6 = [i for i in unvisited_heuristics_6 if unvisited_heuristics_6[i] == current_heuristics_6][0]

        current_weight_1 = unvisited_vertices_1[current_vertex_1]
        current_weight_2 = unvisited_vertices_2[current_vertex_2]
        current_weight_3 = unvisited_vertices_3[current_vertex_3]
        current_weight_4 = unvisited_vertices_4[current_vertex_4]
        current_weight_5 = unvisited_vertices_5[current_vertex_5]
        current_weight_6 = unvisited_vertices_6[current_vertex_6]

        if vertex_heuristics_1.count(float('inf')) == len(vertex_heuristics_1) and vertex_heuristics_2.count(float('inf')) == len(vertex_heuristics_2) and vertex_heuristics_3.count(float('inf')) == len(vertex_heuristics_3) and vertex_heuristics_4.count(float('inf')) == len(vertex_heuristics_4) and vertex_heuristics_5.count(float('inf')) == len(vertex_heuristics_5) and vertex_heuristics_6.count(float('inf')) == len(vertex_heuristics_6):
            break

        if current_vertex_1 == goal_vertex:
            path_1, weights_1 = getShortestPathAndWeight(previous_vertices_1, [0,0])
            done1=True
            print(f"num iterations 1: {i}")
        if current_vertex_2 == goal_vertex:
            path_2, weights_2 = getShortestPathAndWeight(previous_vertices_2, [0,1])
            done2=True
            print(f"num iterations 2: {i}")
        if current_vertex_3 == goal_vertex:
            path_3, weights_3 = getShortestPathAndWeight(previous_vertices_3, [0,2])
            done3=True
            print(f"num iterations 3: {i}")
        if current_vertex_4 == goal_vertex:
            path_4, weights_4 = getShortestPathAndWeight(previous_vertices_4, [1,0])
            done4=True
            print(f"num iterations 4: {i}")
        if current_vertex_5 == goal_vertex:
            path_5, weights_5 = getShortestPathAndWeight(previous_vertices_5, [1,1])
            done5=True
            print(f"num iterations 5: {i}")
        if current_vertex_6 == goal_vertex:
            path_6, weights_6 = getShortestPathAndWeight(previous_vertices_6, [1,2])
            done6=True
            print(f"num iterations 6: {i}")

        if done1 and done2 and done3 and done4 and done5 and done6:
            break
        
        i+=1

        del unvisited_vertices_1[current_vertex_1]
        del unvisited_vertices_2[current_vertex_2]
        del unvisited_vertices_3[current_vertex_3]
        del unvisited_vertices_4[current_vertex_4]
        del unvisited_vertices_5[current_vertex_5]
        del unvisited_vertices_6[current_vertex_6]
        del unvisited_heuristics_1[current_vertex_1]
        del unvisited_heuristics_2[current_vertex_2]
        del unvisited_heuristics_3[current_vertex_3]
        del unvisited_heuristics_4[current_vertex_4]
        del unvisited_heuristics_5[current_vertex_5]
        del unvisited_heuristics_6[current_vertex_6]
        if create_video:
            current_vertex_x_1, current_vertex_y_1 = coordinates[current_vertex_1]
            current_vertex_x_2, current_vertex_y_2 = coordinates[current_vertex_2]
            current_vertex_x_3, current_vertex_y_3 = coordinates[current_vertex_3]
            current_vertex_x_4, current_vertex_y_4 = coordinates[current_vertex_4]
            current_vertex_x_5, current_vertex_y_5 = coordinates[current_vertex_5]
            current_vertex_x_6, current_vertex_y_6 = coordinates[current_vertex_6]

            if(not done1):
                axis[0, 0].scatter(current_vertex_x_1, current_vertex_y_1, s=30, color='deepskyblue', zorder=2)
            if(not done2):
                axis[0, 1].scatter(current_vertex_x_2, current_vertex_y_2, s=30, color='deepskyblue', zorder=2)
            if(not done3):
                axis[0, 2].scatter(current_vertex_x_3, current_vertex_y_3, s=30, color='deepskyblue', zorder=2)
            if(not done4):
                axis[1, 0].scatter(current_vertex_x_4, current_vertex_y_4, s=30, color='deepskyblue', zorder=2)
            if(not done5):
                axis[1, 1].scatter(current_vertex_x_5, current_vertex_y_5, s=30, color='deepskyblue', zorder=2)
            if(not done6):
                axis[1, 2].scatter(current_vertex_x_6, current_vertex_y_6, s=30, color='deepskyblue', zorder=2)
            plt.draw()
            plt.pause(0.01)
            writer.grab_frame()

        if(not(done1)):
            current_vertex_edges_1 = edges[current_vertex_1]
            
            for edge in current_vertex_edges_1:
                edge_head = edge[0]
                edge_weight = edge[1]
                
                if edge_head in unvisited_vertices_1:
                    new_vertex_weight = current_weight_1+edge_weight
                    new_vertex_heuristic_weight = new_vertex_weight + heuristic[edge_head]*0
                    
                    if new_vertex_weight < unvisited_vertices_1[edge_head]:
                        unvisited_vertices_1[edge_head] = new_vertex_weight
                    
                    if new_vertex_heuristic_weight < unvisited_heuristics_1[edge_head]:
                        unvisited_heuristics_1[edge_head] = new_vertex_heuristic_weight
                        previous_vertices_1[edge_head] = current_vertex_1
                
                if create_video:
                    current_vertex_x, current_vertex_y = coordinates[current_vertex_1]
                    head_x, head_y = coordinates[edge_head]
                    line = lines.Line2D([current_vertex_x, head_x], [current_vertex_y, head_y], color='aqua', zorder=0)
                    axis[0, 0].add_line(line)
                    
        if(not(done2)):
            current_vertex_edges_2 = edges[current_vertex_2]

            for edge in current_vertex_edges_2:
                edge_head = edge[0]
                edge_weight = edge[1]
                
                if edge_head in unvisited_vertices_2:
                    new_vertex_weight = current_weight_2+edge_weight
                    new_vertex_heuristic_weight = new_vertex_weight + heuristic[edge_head]*1
                    
                    if new_vertex_weight < unvisited_vertices_2[edge_head]:
                        unvisited_vertices_2[edge_head] = new_vertex_weight
                    
                    if new_vertex_heuristic_weight < unvisited_heuristics_2[edge_head]:
                        unvisited_heuristics_2[edge_head] = new_vertex_heuristic_weight
                        previous_vertices_2[edge_head] = current_vertex_2
                
                if create_video:
                    current_vertex_x, current_vertex_y = coordinates[current_vertex_2]
                    head_x, head_y = coordinates[edge_head]
                    line = lines.Line2D([current_vertex_x, head_x], [current_vertex_y, head_y], color='aqua', zorder=0)
                    axis[0, 1].add_line(line)
                
        if(not(done3)):
            current_vertex_edges_3 = edges[current_vertex_3]

            for edge in current_vertex_edges_3:
                edge_head = edge[0]
                edge_weight = edge[1]
                
                if edge_head in unvisited_vertices_3:
                    new_vertex_weight = current_weight_3+edge_weight
                    new_vertex_heuristic_weight = new_vertex_weight + heuristic[edge_head]*2
                    
                    if new_vertex_weight < unvisited_vertices_3[edge_head]:
                        unvisited_vertices_3[edge_head] = new_vertex_weight

                    if new_vertex_heuristic_weight < unvisited_heuristics_3[edge_head]:
                        unvisited_heuristics_3[edge_head] = new_vertex_heuristic_weight
                        previous_vertices_3[edge_head] = current_vertex_3
                
                if create_video:
                    current_vertex_x, current_vertex_y = coordinates[current_vertex_3]
                    head_x, head_y = coordinates[edge_head]
                    line = lines.Line2D([current_vertex_x, head_x], [current_vertex_y, head_y], color='aqua', zorder=0)
                    axis[0, 2].add_line(line)
                
        if(not(done4)):
            current_vertex_edges_4 = edges[current_vertex_4]

            for edge in current_vertex_edges_4:
                edge_head = edge[0]
                edge_weight = edge[1]
                
                if edge_head in unvisited_vertices_4:
                    new_vertex_weight = current_weight_4+edge_weight
                    new_vertex_heuristic_weight = new_vertex_weight + heuristic[edge_head]*3
                    
                    if new_vertex_weight < unvisited_vertices_4[edge_head]:
                        unvisited_vertices_4[edge_head] = new_vertex_weight
                    
                    if new_vertex_heuristic_weight < unvisited_heuristics_4[edge_head]:
                        unvisited_heuristics_4[edge_head] = new_vertex_heuristic_weight
                        previous_vertices_4[edge_head] = current_vertex_4
                
                if create_video:
                    current_vertex_x, current_vertex_y = coordinates[current_vertex_4]
                    head_x, head_y = coordinates[edge_head]
                    line = lines.Line2D([current_vertex_x, head_x], [current_vertex_y, head_y], color='aqua', zorder=0)
                    axis[1, 0].add_line(line)
        
        if(not(done5)):
            current_vertex_edges_5 = edges[current_vertex_5]

            for edge in current_vertex_edges_5:
                edge_head = edge[0]
                edge_weight = edge[1]
                
                if edge_head in unvisited_vertices_5:
                    new_vertex_weight = current_weight_5+edge_weight
                    new_vertex_heuristic_weight = new_vertex_weight + heuristic[edge_head]*4
                    
                    if new_vertex_weight < unvisited_vertices_5[edge_head]:
                        unvisited_vertices_5[edge_head] = new_vertex_weight
                    
                    if new_vertex_heuristic_weight < unvisited_heuristics_5[edge_head]:
                        unvisited_heuristics_5[edge_head] = new_vertex_heuristic_weight
                        previous_vertices_5[edge_head] = current_vertex_5
                
                if create_video:
                    current_vertex_x, current_vertex_y = coordinates[current_vertex_5]
                    head_x, head_y = coordinates[edge_head]
                    line = lines.Line2D([current_vertex_x, head_x], [current_vertex_y, head_y], color='aqua', zorder=0)
                    axis[1, 1].add_line(line)

        if(not(done6)):
            current_vertex_edges_6 = edges[current_vertex_6]
            
            for edge in current_vertex_edges_6:
                edge_head = edge[0]
                edge_weight = edge[1]
                
                if edge_head in unvisited_vertices_6:
                    new_vertex_weight = current_weight_6+edge_weight
                    new_vertex_heuristic_weight = new_vertex_weight + heuristic[edge_head]*5
                    
                    if new_vertex_weight < unvisited_vertices_6[edge_head]:
                        unvisited_vertices_6[edge_head] = new_vertex_weight
                    
                    if new_vertex_heuristic_weight < unvisited_heuristics_6[edge_head]:
                        unvisited_heuristics_6[edge_head] = new_vertex_heuristic_weight
                        previous_vertices_6[edge_head] = current_vertex_6
                
                if create_video:
                    current_vertex_x, current_vertex_y = coordinates[current_vertex_6]
                    head_x, head_y = coordinates[edge_head]
                    line = lines.Line2D([current_vertex_x, head_x], [current_vertex_y, head_y], color='aqua', zorder=0)
                    axis[1, 2].add_line(line)

        if create_video:
            plt.show(block=False)
            plt.pause(0.01)
            writer.grab_frame()

    return path_1, weights_1, path_2, weights_2, path_3, weights_3, path_4, weights_4, path_5, weights_5, path_6, weights_6

def getShortestPathAndWeight(previous_vertices, subplot):
    shortest_path = [goal_vertex]
    current_vertex = goal_vertex
    weight_path = []
    current_weight = 0

    while current_vertex != start_vertex:
        current_vertex_x, current_vertex_y = coordinates[current_vertex]
        current_vertex = previous_vertices[current_vertex]

        shortest_path.insert(0, current_vertex)
        if create_video:
            previous_vertex_x, previous_vertex_y = coordinates[current_vertex]
            axis[subplot[0], subplot[1]].scatter(current_vertex_x, current_vertex_y, s=30, color='lime', zorder=3)
            line = lines.Line2D([current_vertex_x, previous_vertex_x], [current_vertex_y, previous_vertex_y], color='lime', zorder=4)
            axis[subplot[0], subplot[1]].add_line(line)
            plt.draw()
            writer.grab_frame()
            plt.pause(0.01)
    
    current_vertex = None
    current_weight = 0
    for i in range(len(shortest_path)-1):
        weight_path.append(round(current_weight,2))
        current_vertex = shortest_path[i]
        next_vertex = shortest_path[i+1]
        current_edges = edges[current_vertex]
        for edge in current_edges:
            if edge[0] == next_vertex:
                current_weight+= edge[1]
    
    weight_path.append(round(current_weight,2))
    return shortest_path, weight_path
    
def createOutputFile(shortest_paths, weight_paths):
    output_file = open("017856202.txt", "w")
    output_string = ""
    for i in range(len(shortest_paths)):
        shortest_path_string = ""
        weight_path_string = ""

        for vertex in shortest_paths[i]:
            shortest_path_string += str(vertex) + " "
        
        for weight in weight_paths[i]: 
            weight_path_string += str(weight) + " "
        
        if (i == 0):
            output_string = shortest_path_string[:-1] + "\n" + weight_path_string[:-1]
        else:
            output_string += "\n" + shortest_path_string[:-1] + "\n" + weight_path_string[:-1]
    
    output_file.write(output_string)
    output_file.close()

def createGraph():
    graph = {}

    for edge in edges.items():
        tail_coordinate = coordinates[edge[0]]
        
        for head in edge[1]:
            head_coordinate = head[0]

            if tail_coordinate in graph:
                graph[tail_coordinate].append(coordinates[head_coordinate])

            else:
                graph[tail_coordinate] = [coordinates[head_coordinate]]
    return graph

def plotGraph(unvisited_graph):
    unvisited_vertices = list(unvisited_graph.keys())
    graph_edges = []
    
    for vertex, edge in unvisited_graph.items():
        for neighbor in edge:
            graph_edges.append((vertex, neighbor))

    # Draw edges
    for (vertex1, vertex2) in graph_edges:
        x_coords = [vertex1[0], vertex2[0]]
        y_coords = [vertex1[1], vertex2[1]]
        axis[0, 0].plot(x_coords, y_coords, 'grey', linestyle='-', linewidth=1, zorder=0)
        axis[0, 0].set_title('Dijkstra\'s')
        axis[0, 1].plot(x_coords, y_coords, 'grey', linestyle='-', linewidth=1, zorder=0)
        axis[0, 1].set_title('A*: ε=1')
        axis[0, 2].plot(x_coords, y_coords, 'grey', linestyle='-', linewidth=1, zorder=0)
        axis[0, 2].set_title('A*: ε=2')
        axis[1, 0].plot(x_coords, y_coords, 'grey', linestyle='-', linewidth=1, zorder=0)
        axis[1, 0].set_title('A*: ε=3')
        axis[1, 1].plot(x_coords, y_coords, 'grey', linestyle='-', linewidth=1, zorder=0)
        axis[1, 1].set_title('A*: ε=4')
        axis[1, 2].plot(x_coords, y_coords, 'grey', linestyle='-', linewidth=1, zorder=0)
        axis[1, 2].set_title('A*: ε=5')
        
    # Draw vertices
    x_coords, y_coords = zip(*unvisited_vertices)
    axis[0, 0].scatter(x_coords, y_coords, s=30, color='black', zorder=1)
    axis[0, 1].scatter(x_coords, y_coords, s=30, color='black', zorder=1)
    axis[0, 2].scatter(x_coords, y_coords, s=30, color='black', zorder=1)
    axis[1, 0].scatter(x_coords, y_coords, s=30, color='black', zorder=1)
    axis[1, 1].scatter(x_coords, y_coords, s=30, color='black', zorder=1)
    axis[1, 2].scatter(x_coords, y_coords, s=30, color='black', zorder=1)

    # Make start and goal vertices different from the rest
    start_x_coords = [coordinates[start_vertex][0]]
    start_y_coords = [coordinates[start_vertex][1]]
    axis[0, 0].scatter(start_x_coords, start_y_coords, s=30, color='lime', zorder=3)
    axis[0, 1].scatter(start_x_coords, start_y_coords, s=30, color='lime', zorder=3)
    axis[0, 2].scatter(start_x_coords, start_y_coords, s=30, color='lime', zorder=3)
    axis[1, 0].scatter(start_x_coords, start_y_coords, s=30, color='lime', zorder=3)
    axis[1, 1].scatter(start_x_coords, start_y_coords, s=30, color='lime', zorder=3)
    axis[1, 2].scatter(start_x_coords, start_y_coords, s=30, color='lime', zorder=3)
    end_x_coords = [coordinates[goal_vertex][0]]
    end_y_coords = [coordinates[goal_vertex][1]]
    axis[0, 0].scatter(end_x_coords, end_y_coords, s=30, color='red', zorder=3)
    axis[0, 1].scatter(end_x_coords, end_y_coords, s=30, color='red', zorder=3)
    axis[0, 2].scatter(end_x_coords, end_y_coords, s=30, color='red', zorder=3)
    axis[1, 0].scatter(end_x_coords, end_y_coords, s=30, color='red', zorder=3)
    axis[1, 1].scatter(end_x_coords, end_y_coords, s=30, color='red', zorder=3)
    axis[1, 2].scatter(end_x_coords, end_y_coords, s=30, color='red', zorder=3)

    axis[0, 0].set_aspect('equal')
    axis[0, 1].set_aspect('equal')
    axis[0, 2].set_aspect('equal')
    axis[1, 0].set_aspect('equal')
    axis[1, 1].set_aspect('equal')
    axis[1, 2].set_aspect('equal')

    # Show plot
    plt.grid(False)
    fig.canvas.draw()
    plt.pause(0.1)
    writer.grab_frame()
    return axis

def main() :
    readInputFile()
    readCoordinateFile()
    heuristic = calculateHeuristic(goal_vertex)

    if create_video:
        with writer.saving(fig, "017856202.mp4", 100): 
            path_1, weights_1, path_2, weights_2, path_3, weights_3, path_4, weights_4, path_5, weights_5, path_6, weights_6 = a_star(heuristic)
            
    else:
        path_1, weights_1, path_2, weights_2, path_3, weights_3, path_4, weights_4, path_5, weights_5, path_6, weights_6 = a_star(heuristic)
    createOutputFile([path_1, path_2, path_3, path_4, path_5, path_6], [weights_1, weights_2, weights_3, weights_4, weights_5, weights_6])

main()