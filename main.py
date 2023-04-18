from create_env import Environment
from create_nx_graph import  NetworkxNeighbours
import networkx as nx
import matplotlib.pyplot as plt
from create_visibility import Visibility
from find_hamiltonain_path import HamiltonianPathFinder

def main():
    #Create Environment
    input_file_name = 'input_file'
    new_environment = Environment(input_file_name)
    new_environment.parse_obstacles()
    new_environment.find_vertical_lines()

    #Get parameters for Networkx graph of the environment
    nx_points = new_environment.get_env_graph_points()
    nx_obstacles= new_environment.get_env_new_obstacles()

    number_of_neigbours=9
    #Create the Networx_x Graph
    env_nx_graph = NetworkxNeighbours(nx_points, nx_obstacles, number_of_neigbours)
    G=env_nx_graph.get_neighbours()
    plt.show()

    # Create the visibility Polygons
    env_visibility = Visibility(nx_obstacles,graph_points=nx_points,env=new_environment.env,obs1=new_environment.hole1
                                    ,obs2=new_environment.hole2,obs3=new_environment.hole3)
    env_visibility.calculate_visibility_polygons()
    sectorcoords_dict,max_sector_vpa,vertex_sequence = env_visibility.calculate_vertexsquence_from_freespace()

    #Find Hamiltonain Path
    hamiltonain_path_of_env = HamiltonianPathFinder(G,vertex_sequence,nx_points,max_sector_vpa)
    adj_matrix = hamiltonain_path_of_env._create_adj_matrix()
    hamiltonain_path_with_ratio_graph,hamiltonain_path_with_ratio,hamiltonain_path_with_ratio_edge_distance = hamiltonain_path_of_env._get_hamiltonian_path_with_ratio_as_weights(adj_matrix)
    #hamiltonain_path_with_length_graph,hamiltonain_path_with_length,hamiltonain_path_with_length_edge_distance = hamiltonain_path_of_env._get_hamiltonian_path_with_length()
    hamiltonain_path_of_env.draw_hamiltonian_cycle(hamiltonain_path_with_ratio)

    sub_path_dict_from_main_graph = hamiltonain_path_of_env.draw_hamiltonian_circles_on_main_graph(hamiltonain_path_with_ratio,
                                                        sectorcoords_dict,obs1 = new_environment.hole1, obs2 = new_environment.hole2, obs3 = new_environment.hole3)

    plt.show()



if __name__ == "__main__":
        main()