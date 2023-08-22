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
    Linelist = new_environment.LineList

    #Get parameters for Networkx graph of the environment
    nx_points = new_environment.get_env_graph_points()
    nx_obstacles= new_environment.get_env_new_obstacles()

    # Create the Networx_x Graph
    number_of_neigbours=9
    env_nx_graph = NetworkxNeighbours(nx_points, nx_obstacles, number_of_neigbours)
    G=env_nx_graph.get_neighbours()
    plt.show()


    # Create the visibility Polygons
    env_visibility = Visibility(nx_obstacles,graph_points=nx_points,env=new_environment.env,obs1=new_environment.hole1
                                    ,obs2=new_environment.hole2,obs3=new_environment.hole3)
    env_visibility.calculate_visibility_polygons()
    sectorcoords_dict,max_sector_vpa,vertex_sequence = env_visibility.calculate_vertexsquence_from_freespace()



    #Find Hamiltonain Path
    hamiltonian_path_of_env = HamiltonianPathFinder(G,vertex_sequence,nx_points,max_sector_vpa)
    adj_matrix = hamiltonian_path_of_env._create_adj_matrix()
    hamiltonian_path_with_ratio_graph,hamiltonian_path_with_ratio,hamiltonian_path_with_ratio_edge_distance = hamiltonian_path_of_env._get_hamiltonian_path_with_ratio_as_weights(adj_matrix)
    #hamiltonain_path_with_length_graph,hamiltonain_path_with_length,hamiltonain_path_with_length_edge_distance = hamiltonain_path_of_env._get_hamiltonian_path_with_length()
    hamiltonian_path_of_env.draw_hamiltonian_cycle(hamiltonian_path_with_ratio)


    #Draw Hamiltonaina path on main graph
    sub_path_dict_from_main_graph = hamiltonian_path_of_env.draw_hamiltonian_circles_on_main_graph(hamiltonian_path_with_ratio,
                                                        sectorcoords_dict,obs1 = new_environment.hole1, obs2 = new_environment.hole2,
                                                        obs3 = new_environment.hole3,LineList=Linelist)
    plt.show()

#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #Divide the paths into N number of robots
    print(sub_path_dict_from_main_graph)

    # Create a dictionary to store edges and weights
    edges_with_weights = {}

    # Iterate through edges of the nx graph and store them in the dictionary
    for edge in G.edges(data=True):
        source, target, attributes = edge
        weight = attributes.get('weight')
        edges_with_weights[(source, target)] = weight

    # Print the dictionary
    # print("Edges with weights:")
    # for edge, weight in edges_with_weights.items():
    #     print(f"Edge {edge} - Weight: {weight}")

    # Calculate the total path distance with adj matrix and also creates a list of sub paths
    total_distance = 0
    sub_path_list = []
    for start_node, path in sub_path_dict_from_main_graph.items():
        for i in range(len(path) - 1):
            source = path[i]
            target = path[i + 1]
            total_distance += env_nx_graph.get_adj_matrix()[source][target]
            # print(f"Edge: {(source,target)} edge weight: {env_nx_graph.get_adj_matrix()[source][target]}  Current distance covered: {total_distance} ")

            sub_path_list.append(source)
    sub_path_list.append(target)

    # print(f"Total adj matrix distance covered: {total_distance}")


    # Calculate the total path distance with graph weights
    # total_distance = 0
    # for start_node, path in sub_path_dict_from_main_graph.items():
    #     for i in range(len(path) - 1):
    #         source = path[i]
    #         target = path[i + 1]
    #         edge_weight = edges_with_weights.get((source, target), edges_with_weights.get((target, source)))
    #
    #         total_distance += edge_weight
    #         print(f"Edge: {(source,target)} edge weight: {edge_weight}  Current distance covered: {total_distance} ")
    #
    # print(f"Total distance covered: {total_distance}")

    # for start_node, path in sub_path_dict_from_main_graph.items():
    #     for node in path:
    #         if start_node not in sub_path_list:
    #             sub_path_list.append(node)
    #
    # sub_path_list.append(sub_path_list[0])
    print(sub_path_list)

    total_distance = 0
    for i in range(len(sub_path_list) - 1):
        print(f"Roboti {i} ")
        source = sub_path_list[i]
        target = sub_path_list[i + 1]
        total_distance += env_nx_graph.get_adj_matrix()[source][target]
    #     print(
    #         f"Edge: {(source, target)} edge weight: {env_nx_graph.get_adj_matrix()[source][target]}  Current distance covered: {total_distance} ")
    # print(f"Total new adj matrix distance covered: {total_distance}")



    # Divide the paths into N number of robots
    number_of_divisions = 4
    avg_distance = total_distance/number_of_divisions
    division_total_distance_list=[]
    robots_patrol_edges_dict={}
    division =1
    current_division_total_distance=0
    temp_robot_patrol_edge_list = []
    #initalize current robot start node
    count = 0
    # print (len(sub_path_list)-1)
    # print (len(sub_path_list)+(number_of_divisions-1))
    for i in range(len(sub_path_list) + (number_of_divisions - 2)):
        print(f"Roboti {i} ")
        #for division in range(1, number_of_robots + 1):
        source = sub_path_list[count]
        target = sub_path_list[count + 1]
        edge_weight = env_nx_graph.get_adj_matrix()[source][target]
        current_division_total_distance += edge_weight

        # print(f"Robotcount {count} , divison{division}")
        # print(f"Source {source} , Target {target}")
        # print(current_division_total_distance, edge_weight , avg_distance)
        count =count+1
        if(current_division_total_distance<=(division*avg_distance)):
            temp_robot_patrol_edge_list.append((source,target)) #edges that current division robot will visit

        else:
            count=count-1
            robots_patrol_edges_dict[division] = temp_robot_patrol_edge_list
            division_total_distance_list.append(current_division_total_distance)
            division = division+1
            current_division_total_distance=current_division_total_distance-edge_weight
            # print (division, temp_robot_patrol_edge_list)
            temp_robot_patrol_edge_list = []
                #change the start_node to current target
    if (division == number_of_divisions):
        robots_patrol_edges_dict[division] = temp_robot_patrol_edge_list
        division_total_distance_list.append(current_division_total_distance)
        # print(division, temp_robot_patrol_edge_list)
    print (robots_patrol_edges_dict)
    #robots_patrol_edges_dict[robot] = temp_robot_patrol_edge_list

    hamiltonian_path_of_env.draw_division_on_main_graph(divison_dict=robots_patrol_edges_dict,obs1 = new_environment.hole1, obs2 = new_environment.hole2,
                                                        obs3 = new_environment.hole3,LineList=Linelist)
    plt.show()








if __name__ == "__main__":
        main()