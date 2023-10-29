import subprocess
import numpy as np
import matplotlib.pyplot as plt
import ast

def get_cpp_output():
    result = subprocess.run(['./Dijkstra'], stdout=subprocess.PIPE)
    output = result.stdout.decode().strip()
    return output

def plot_graph(input):

    lists = input.split(",r")[:-1]    
    
    parsed_data = [ast.literal_eval(item) for item in lists] # Parse the input strings into lists of lists
    map = np.array(parsed_data) # Convert the lists into a NumPy array

    map_dim = len(lists) - 1

    # plot the map
    map2 = np.flipud(np.transpose(map,(1,0,2)))
    plt.imshow(map2,cmap='gray',vmin=0, vmax=1)  # grayscale: gray / rgb: brg

    ax = plt.gca()
    ax.set_xticks(np.arange(-.5, map_dim, 1), minor=True)
    ax.set_yticks(np.arange(-.5, map_dim, 1), minor=True)
    ax.grid(which='minor', color='w', linestyle='-', linewidth=0.1)

    plt.show()


if __name__ == "__main__":
    output_map = get_cpp_output()
    plot_graph(output_map)
