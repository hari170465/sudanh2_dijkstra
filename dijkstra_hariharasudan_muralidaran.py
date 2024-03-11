import cv2
import numpy as np


def create_map():
    map_image = np.zeros((500, 1200, 3), dtype=np.uint8)
    for x in range (1200):
        for y in range (500):
        # First 2 obstacles in red
            if(100 <= x <= 175 and 0 <= y <= 400):
                cv2.circle(map_image, (x, y), 2, (0, 0, 255), -1)
            elif(275 <= x <= 350 and 100 <= y <= 500):
                cv2.circle(map_image, (x, y), 2, (0, 0, 255), -1)

            # Stapler pin in red
            elif(900 <= x <= 1100 and 50 <= y <= 130):
                cv2.circle(map_image, (x, y), 2, (0, 0, 255), -1)
            elif(1010 <= x <= 1100 and 130 <= y <= 370):
                cv2.circle(map_image, (x, y), 2, (0, 0, 255), -1)
            elif(900 <= x <= 1100 and 370 <= y <= 450):
                cv2.circle(map_image, (x, y), 2, (0, 0, 255), -1)

            # the obstacles in red
            elif(95 <= x <= 180 and 0 <= y <= 405):
                cv2.circle(map_image, (x, y), 2, (255, 0, 0), -1)
            elif(270 <= x <= 355 and 95 <= y <= 500):
                cv2.circle(map_image, (x, y), 2, (255, 0, 0), -1)
            elif(895 <= x <= 1105 and 45 <= y <= 135):
                cv2.circle(map_image, (x, y), 2, (255, 0, 0), -1)
            elif(1005 <= x <= 1105 and 130 <= y <= 370):
                cv2.circle(map_image, (x, y), 2, (255, 0, 0), -1)
            elif(895 <= x <= 1105 and 365 <= y <= 455):
                cv2.circle(map_image, (x, y), 2, (255, 0, 0), -1)

    # Obstacle 3 - hexagon (5mm clearance accounted for)
    hex_pt_1 = np.array([
        [650, 94],
        [785, 172],
        [785, 328.9],
        [650, 405],
        [515, 328.9],
        [515, 172]
    ], np.int32)

    cv2.fillPoly(map_image, [hex_pt_1], (255, 0, 0))
    hex_pt_2 = np.array([
        [650, 100],
        [781, 175],
        [781, 325],
        [651, 400],
        [520, 325],
        [520, 175]
    ], np.int32)
    cv2.fillPoly(map_image, [hex_pt_2], (0, 0, 255))
    
    return map_image


fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video_out = cv2.VideoWriter('dijkstra_hariharasudan_muralidaran_output.mp4', fourcc, 10.0, (1200, 500))

# Find the near by neighbors and their respective cost values
def find_neighbors(x, y, width, height, map_image):
    neighbor = []
    dia_cost = 1.4
    ortho_cost = 1
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    # Iterating through possible movement directions to find valid neighbors
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height:
            # print(np.array(map_image[ny,nx]))
            if not (map_image[ny, nx][0] == 255 and map_image[ny, nx][1] == 0 and map_image[ny, nx][2] == 0):
                step_cost = dia_cost if dx != 0 and dy != 0 else ortho_cost
                neighbor_index = ny * width + nx  # Converting back to 1D 
                neighbor.append((neighbor_index, step_cost))  # Appending the neighbor index and the cost to a variable and return it 

    return neighbor

# Visualizing the final path
def visualize_path(map_image, path):
    for index in path:
        x = (index % 1200) 
        y = (index // 1200)
        cv2.circle(map_image, (x, y), 2, (0, 255, 0), -1)  # Green color for path
        video_out.write(map_image)
        
# Visualizing the nodes discovered
def visualize_nodes(close, map_image):
    # Visualization of closed nodes
    interval=100
    for i, current_node_1 in enumerate(close):
        x_1, y_1 = current_node_1[0], current_node_1[1]
        cv2.circle(map_image, (x_1, y_1), 1, (0, 255, 255), -1)  # Yellow for explored node
        if i % interval == 0:
            video_out.write(map_image)  # Writing the frame with intervals to reduce computation time 

# dijkstra algorithim 
def dijkstra(map_image, start_x, start_y, goal_x, goal_y, width, height):
    # Initializing the variables to implement the dijkstra algorithim
    start_index = start_y * width + start_x
    # print(start_index)
    goal_index = goal_y * width + goal_x
    # print(goal_index)
    open_list = []
    closed_list = set()
    parents = {}
    g_costs = {start_index: 0}
    open_list.append((start_index, 0))
    path_found = False
    shortest_path = []
    visual=[]

    while open_list:
        open_list.sort(key=lambda x: x[1])
        current_node, _ = open_list.pop(0)  # Getting the first element from the sorted list to processes next

        closed_list.add(current_node)

        x, y = (current_node % width), (current_node // width)  # Converting the current node index to its respective x and y quantities
        visual.append([x,y])   # Needed to process the node visualiztion

        # Checking if the current node index matches with the goal node index
        if current_node == goal_index: 
            path_found = True
            break
        
        # Calling the find neighbor function to find the neighbors and the costs associated
        neighbors = find_neighbors(x,y, width, height, map_image)

        for neighbor_index, step_cost in neighbors:
            if neighbor_index in closed_list:
                continue
            g_cost = g_costs[current_node] + step_cost  #updating the current nodes g cost

            # Checking if the neighbor is in open_list
            in_open_list = False
            for idx, element in enumerate(open_list):
                if element[0] == neighbor_index:
                    in_open_list = True
                    break

            # if neighbor is already in open_list
            if in_open_list:
                if g_cost < g_costs[neighbor_index]:
                    # Update the node's g_cost inside g_costs
                    g_costs[neighbor_index] = g_cost
                    parents[neighbor_index] = current_node
                    # Update the node's g_cost inside open_list
                    open_list[idx] = [neighbor_index, g_cost]

            # If neighbor is not in open_list
            else:
                # Set the node's g_cost inside g_costs
                g_costs[neighbor_index] = g_cost
                parents[neighbor_index] = current_node
                # Add neighbor to open_list
                open_list.append([neighbor_index, g_cost])
    if not path_found:
        return shortest_path,visual
    # to find the shortest part
    if path_found:
        node = goal_index
        shortest_path.append(goal_index)  # Adding goal node to the shortest path found(in the end)

        while node != start_index:
            shortest_path.append(node)
            node = parents[node]
        shortest_path.append(start_index)  # Adding start node to the shortest path found(in the begining)
        shortest_path.reverse()  # Reversing it and returning it

    return shortest_path,visual

# Main function
def main():
    # Creating the map
    map_image = create_map()
    # print(np.array(map_image[120,120]))

    # Getting the user inputs
    start_x = int(input("enter the x value of the start point:"))
    start_y = int(input("enter the y value of the start point:"))
    goal_x = int(input("enter the x value of the goal point:"))
    goal_y = int(input("enter the y value of the goal point:"))

    start_point = (-1200+start_x, 500-start_y)  #start point
    goal_point = (-1200+goal_x, 500-goal_y)  #goal point

    # Conditions to check if the user input is inside an obstacle
    if(map_image[(500-goal_y,-1200+goal_x)][0] == 255 and map_image[(500-goal_y,-1200+goal_x)][1] == 0 and map_image[(500-goal_y,-1200+goal_x)][2] == 0):
        print("goal point in an obstacle")
    elif(map_image[(500-start_y, -1200+start_x)][0] == 255 and map_image[(500-start_y, -1200+start_x)][1] == 0 and map_image[(500-start_y, -1200+start_x)][2] == 0):
        print("goal point in an obstacle")

    else:
        # Function to run Dijkstra's algorithm
        path, close= dijkstra(map_image, start_point[0], start_point[1], goal_point[0], goal_point[1], map_image.shape[1], map_image.shape[0])
        # Drawing the nodes progession
        visualize_nodes(close,map_image)
        # Drawing the final path
        visualize_path(map_image, path)
        # showing the result
        cv2.imshow('Path on Map', map_image)
        video_out.release()
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()