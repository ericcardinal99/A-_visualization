# A* Algorithm Visualization
## Implementation
The first step of the implementation of A* Algorithm was to convert the input.txt and coords.txt files into useful data structures. From the input.txt file, the total number of vertices, starting vertex and goal vertex are saved. Then, the edges are saved in a dictionary. The format of this dictionary is as follows:
	
	{
           tail vertex 1: [
                (head vertex 1, vertex weight), 
                (head vertex 2, vertex weight), 
                ...],
	     tail vertex 2: [
                (head vertex 1, vertex weight), 
                (head vertex 2, vertex weight), 
                ...],
	     ...
      }


From the coord.txt file, the coordinates dictionary is created. The format of this dictionary is as follows:
	
	{
		vertex 1: (x_coordinate 1, y_coordinate 1),
		vertex 2: (x_coordinate 2, y_coordinate 2),
		...
      }


For the A* algorithm, a heuristic is needed for each node representing its distance to the goal node. With the coordinates dictionary, I calculated the heuristic as follows:

    heuristic = {}
    goal_vertex= coordinates[goal_vertex]
    
    for vertex, coordinate in coordinates.items():
        heuristic[vertex] = pow(pow((goal_vertex[0]-coordinate[0]), 2) + pow((goal_vertex[1]-coordinate[1]), 2), 1/2)
    return heuristic

With these dictionaries, along with the starting and goal vertices, the A* algorithm could now be implemented. Wikipedia’s pseudo code was followed for the algorithm (https://en.wikipedia.org/wiki/A*_search_algorithm).


## How to Compile and Run
Assuming python is already installed, the only installation needed to compile and run this code is matplotlib. Following the installation instructions on matplotlib’s official website (https://matplotlib.org/stable/install/index.html), the only commands needed are as follows:

python -m pip install -U pip
python -m pip install -U matplotlib


At the time of submission, the ffmpeg.exe file is up to date and working as expected. If running the file gives an error message related to:

with writer.saving(fig, "017856202.mp4", 100):
OR
writer.grab_frame()


Go to download the latest full ffmpeg build (https://www.gyan.dev/ffmpeg/builds/) and extract the ffmpeg.exe file into the folder where the file python file exists.

When matplotlib and ffmpeg.exe are properly installed, the last step is to run the python file:

python 017856202.py


This will run A* Algorithm Visualization. If the visualization is taking too long and you don’t want to see it anymore, simply change line 6 from:

create_video = True
TO
Create_video = False



## Algorithm Results

Algorithm
Total Cost
Total Number of Iterations
Dijkstra’s
30.14
98
A*
30.14
62
Weighted A*, ε = 2
30.63
30
Weighted A*, ε = 3
31.80
26
Weighted A*, ε = 4
31.80
27
Weighted A*, ε = 5
31.80
26


