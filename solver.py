import sys
import random
import math
import numpy as np
from visualiser import Visualiser
from angle import Angle
from problem_spec import ProblemSpec
from robot_config import make_robot_config_from_ee1, make_robot_config_from_ee2, write_robot_config_list_to_file
from tester import test_config_equality

"""
Template file for you to implement your solution to Assignment 2. Contains a class you can use to represent graph nodes,
and a method for finding a path in a graph made up of GraphNode objects.

COMP3702 2020 Assignment 2 Support Code
"""


class GraphNode:
    """
    Class representing a node in the state graph. You should create an instance of this class each time you generate
    a sample.
    """

    def __init__(self, spec, config):
        """
        Create a new graph node object for the given config.

        Neighbors should be added by appending to self.neighbors after creating each new GraphNode.

        :param spec: ProblemSpec object
        :param config: the RobotConfig object to be stored in this node
        """
        self.spec = spec
        self.config = config
        self.neighbors = []

    def __eq__(self, other):
        return test_config_equality(self.config, other.config, self.spec)

    def __hash__(self):
        return hash(tuple(self.config.points))

    def get_successors(self):
        return self.neighbors

    @staticmethod
    def add_connection(n1, n2):
        """
        Creates a neighbor connection between the 2 given GraphNode objects.

        :param n1: a GraphNode object
        :param n2: a GraphNode object
        """
        n1.neighbors.append(n2)
        n2.neighbors.append(n1)


def find_graph_path(spec, init_node, goal_node):
    """
    This method performs a breadth first search of the state graph and return a list of configs which form a path
    through the state graph between the initial and the goal. Note that this path will not satisfy the primitive step
    requirement - you will need to interpolate between the configs in the returned list.

    You may use this method in your solver if you wish, or can implement your own graph search algorithm to improve
    performance.

    :param spec: ProblemSpec object
    :param init_node: GraphNode object for the initial configuration
    :return: List of configs forming a path through the graph from initial to goal
    """
    # search the graph
    init_container = [init_node]

    # here, each key is a graph node, each value is the list of configs visited on the path to the graph node
    init_visited = {init_node: [init_node.config]}

    while len(init_container) > 0:
        current = init_container.pop(0)

        if test_config_equality(current.config, goal_node.config, spec):
            # found path to goal
            return init_visited[current]

        successors = current.get_successors()
        for suc in successors:
            if suc not in init_visited:
                init_container.append(suc)
                init_visited[suc] = init_visited[current] + [suc.config]

    return None

def generate_angles(num_angles):
    """
    Utility function allowing for generation of a random set of lengths within spec parameters

    Will be between 0 and 360 degrees if is angle on effector attached to grapple point, otherwise between -165 and 165 degrees
    """
    new_angles = []
    for index in range(num_angles):
        new_deg =  degrees=random.randrange(0,361, 1) if index == 0 else random.randrange(-165,165, 1)
        new_angle = Angle(degrees=new_deg)
        new_angles.append(new_angle)
    return new_angles

def generate_lengths(min_lengths, max_lengths):
    """
    Utility function allowing generation of a random set of lengths within spec paramaters
    """
    num_segments = len(min_lengths)
    new_lengths = []
    for i in range(num_segments):
        if min_lengths[i] == max_lengths[i]:
            new_lengths.append(min_lengths[i])
            continue
        new_length = random.uniform(min_lengths[i], max_lengths[i]) # may need some debugging for more advanced configurations
        new_lengths.append(new_length)
    return new_lengths


def generate_sample(spec, grapple_point_index):
    """
    Generates a sample robot config taking a problem spec as input.
    """
    new_angles = generate_angles(spec.num_segments) # num_segments == num_angles 
    new_lengths = generate_lengths(spec.min_lengths, spec.max_lengths)
    
    # for now only implementing grapple points == 1
    if spec.num_grapple_points == 1:
        is_ee1_grappled = spec.initial.ee1_grappled
        robot_config = make_robot_config_from_ee1(spec.grapple_points[0][0], spec.grapple_points[0][1], new_angles, new_lengths, ee1_grappled=is_ee1_grappled, ee2_grappled= not is_ee1_grappled)
    else:
        is_ee1_grappled = True if grapple_point_index == 0 else False #This is essentially hardcoding, fix it if you have time
        new_x = spec.grapple_points[grapple_point_index][0]
        new_y = spec.grapple_points[grapple_point_index][1]
        if is_ee1_grappled:
            robot_config = make_robot_config_from_ee1(new_x, new_y, new_angles, new_lengths, ee1_grappled=is_ee1_grappled, ee2_grappled= not is_ee1_grappled)
        else:   
            robot_config = make_robot_config_from_ee2(new_x, new_y, new_angles, new_lengths, ee1_grappled=is_ee1_grappled, ee2_grappled= not is_ee1_grappled)
    
    return robot_config

def edges_intersect(edge1, edge2):
    """
    Takes two edges and returns a boolean depending on whether they intersect
    """
    comparison_matrix = np.array([
        [edge1[0][0], edge1[0][1], 1],
        [edge1[1][0], edge1[1][1], 1],
        [edge2[0][0], edge2[0][1], 1]
    ])
    determinant_1 = np.linalg.det(comparison_matrix)

    comparison_matrix = np.array([
        [edge1[0][0], edge1[0][1], 1],
        [edge1[1][0], edge1[1][1], 1],
        [edge2[1][0], edge2[1][1], 1]
    ])
    determinant_2 = np.linalg.det(comparison_matrix)

    comparison_matrix = np.array([
        [edge2[0][0], edge2[0][1], 1],
        [edge2[1][0], edge2[1][1], 1],
        [edge1[0][0], edge1[0][1], 1]
    ])
    determinant_3 = np.linalg.det(comparison_matrix)

    comparison_matrix = np.array([
        [edge2[0][0], edge2[0][1], 1],
        [edge2[1][0], edge2[1][1], 1],
        [edge1[1][0], edge1[1][1], 1]
    ])
    determinant_4 = np.linalg.det(comparison_matrix)

    # youll need to implement the special case also

    if determinant_1 * determinant_2 < 0 and determinant_3 * determinant_4 < 0:
        return True

    return False

def is_step_collision(spec, config):
    """
    Determines whether the configuration of the robot provided to the function results in a collision with the edge of the workspace or any of the obstacles on the part of the segments.

    :param spec: ProblemSpec object
    :param config: RobotConfig object
    :return: boolean representing whether the robot configuration results in a collision or not
    """
    for obstacle in spec.obstacles:
        for obstacle_edge in obstacle.edges:
            for segment_edge in config.edges:
                if edges_intersect(obstacle_edge, segment_edge):
                    return True

    workspace_edges = [((0,0),(0,1)), ((0,1),(1,1)), ((1,1), (1,0)), ((1,0), (0,0))]

    for workspace_edge in workspace_edges:
        for segment_edge in config.edges:
            if edges_intersect(workspace_edge, segment_edge):
                return True
                
    return False

def is_connect_angles(angles1, angles2):
    """
    Determines whether two angles are close for the purpose of connecting them in the state graph, returning a boolean value. This function does NOT obey the tolerance rule

    :param angles1: array of Angle objects
    :param angles2: array of Angle objects
    """
    num_angles = len(angles1)
    for i in range(num_angles):
        if abs(angles1[i].in_radians() - angles2[i].in_radians()) > 0.25: # picked somewhat arbitrarily, can fine-tune
            return False
    return True

def is_connect_lengths(lengths1, lengths2):
    """
    Determines whether two lengths are close for the purpose of connecting them in the state graph, returning a boolean value. This function does NOT obey the tolerance rule

    :param lengths1: array of floats
    :param lengths2: array of floats
    """
    num_lengths = len(lengths1)
    for i in range(num_lengths):
        if abs(lengths1[i] - lengths2[i]) > 0.1: # picked somewhat arbitrarily, can fine-tune
            return False
    return True

def vals_within_tolerance(current, goal, tolerance):
    num_vals = len(current)
    for i in range(num_vals):
        if current[i] - goal[i] > tolerance or current[i] - goal[i] < -tolerance:
            return False
    return True

# The above function was bugging out so I used this from tester. Figure out the bug and use 
# your own code, which I always prefer, if you have time.
def test_config_distance(c1, c2, spec):
    """
    Check that the maximum distance between configurations is less than one step.
    :return: True if less than one primitive step apart, false otherwise
    """
    max_ee1_delta = 0
    max_ee2_delta = 0
    for i in range(spec.num_segments):
        if abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians()) > max_ee1_delta:
            max_ee1_delta = abs((c2.ee1_angles[i] - c1.ee1_angles[i]).in_radians())

        if abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians()) > max_ee2_delta:
            max_ee2_delta = abs((c2.ee2_angles[i] - c1.ee2_angles[i]).in_radians())

    # measure leniently - allow compliance from EE1 or EE2
    max_delta = min(max_ee1_delta, max_ee2_delta)

    for i in range(spec.num_segments):
        if abs(c2.lengths[i] - c1.lengths[i]) > max_delta:
            max_delta = abs(c2.lengths[i] - c1.lengths[i])

    if max_delta > spec.PRIMITIVE_STEP + spec.TOLERANCE:
        return False
    return True

def generate_new_vals(current, goal, tolerance):
    num_els = len(current)
    new_vals = []
    for i in range(num_els):
        difference = current[i] - goal[i]
        if difference > tolerance:
            new_vals.append(current[i] - tolerance)
        elif difference < -tolerance:
            new_vals.append(current[i] + tolerance)
        else:
            new_vals.append(current[i])
    return new_vals

#might refactor this into recursive pathfind or somesuch
# needs collision checking
def interpolate_path_segment(spec, start, end, grapple_point_index): 
    primitive_step = spec.PRIMITIVE_STEP 
    interpolated_path = [start]
    
    is_in_tolerance = test_config_distance(start, end, spec)

    is_ee1_grappled = start.ee1_grappled

    while not is_in_tolerance:        
        new_lengths = generate_new_vals(interpolated_path[-1].lengths, end.lengths, primitive_step)
        new_x = spec.grapple_points[grapple_point_index][0]
        new_y = spec.grapple_points[grapple_point_index][1]
        if is_ee1_grappled:
            new_angles = generate_new_vals(interpolated_path[-1].ee1_angles, end.ee1_angles, primitive_step)
            new_config = make_robot_config_from_ee1(new_x, new_y, new_angles, new_lengths, ee1_grappled=is_ee1_grappled, ee2_grappled= not is_ee1_grappled)
        else:   
            new_angles = generate_new_vals(interpolated_path[-1].ee2_angles, end.ee2_angles, primitive_step)
            new_config = make_robot_config_from_ee2(new_x, new_y, new_angles, new_lengths, ee1_grappled=is_ee1_grappled, ee2_grappled= not is_ee1_grappled)
        if is_step_collision(spec, new_config):
            return []
        interpolated_path.append(new_config)
        is_in_tolerance = test_config_distance(new_config, end, spec)
    return interpolated_path

def search_steps_violations(spec, steps):
    num_steps = len(steps)
    primitive_step = spec.PRIMITIVE_STEP
    violations = []
    for i in range(num_steps - 1):
        step1 = steps[i]
        step2 = steps[i+1]
        if test_config_distance(step1, step2, spec) is False:
            violations.append({"steps":(step1, step2), "split_index": (i+1)})
    
    return violations

def calc_distance_between_points(point1, point2):
    return math.sqrt(((point1[0]-point2[0])**2) + ((point1[1]-point2[1])**2))


def find_closest_node_in_adjacent_set(spec, adjacent_node_set,grapple_point_index):
    current_grapple_point = spec.grapple_points[grapple_point_index]
    min_distance = 999
    closest_node = None
    for node in adjacent_node_set:
        config_end_point = node.config.points[0] if node.config.ee2_grappled else node.config.points[-1]
        distance_to_grapple_point = calc_distance_between_points(current_grapple_point, config_end_point)
        if distance_to_grapple_point < min_distance:
            min_distance = distance_to_grapple_point
            closest_node = node
    return closest_node

def connect_node_to_neighbours(new_node, graph_node_list):

    for node in graph_node_list:
        node_angles, new_node_angles, node_lengths, new_node_lengths = node.config.ee1_angles, new_node.config.ee1_angles, node.config.lengths, new_node.config.lengths
        if is_connect_angles(node_angles, new_node_angles) and is_connect_lengths(node_lengths, new_node_lengths):
            GraphNode.add_connection(node, new_node)
    return new_node

def interpolate_path(spec, path, grapple_point_index):
    num_steps = len(path)
    interpolated_path = []
    for i in range(num_steps - 1):
        new_path_segment = interpolate_path_segment(spec, path[i], path[i+1], grapple_point_index)
        interpolated_path.extend(new_path_segment)
    interpolated_path.append(path[-1])
    
    return interpolated_path

def detect_and_correct_violations(spec, path):
    new_path = []
    violations = search_steps_violations(spec, path)
    num_violations = len(violations) 
    if num_violations > 0:
        split_arrs = []
        current_violations_index = violations[0]["split_index"] 
        split_arrs.append(path[0:current_violations_index])
        for i in range(num_violations - 1):
            new_violations_index = violations[i+1]["split_index"]
            split_arrs.append(path[current_violations_index:new_violations_index])
            current_violations_index = new_violations_index
        split_arrs.append(path[current_violations_index:])
        for i in range(len(split_arrs) - 1):
            new_path.extend(split_arrs[i])
            # find a new path
            start_node = GraphNode(spec, split_arrs[i][-1])
            end_node = GraphNode(spec, split_arrs[i+1][0])
            graph_nodes=[[start_node, end_node]]
            path_segment = generate_steps(spec, 5000, graph_nodes, start_node, end_node, num_grapple_points = 1)
            new_path.extend(path_segment)
        new_path.extend(split_arrs[-1])
        return new_path
    return path

        # path = interpolate_recursively(path, violations) violations will have index information

def generate_steps(spec, sample_size, graph_nodes, init_node, goal_node, num_grapple_points = 1):
    steps = []
    isPathFound = False
    while isPathFound is False:
        n = 0
        while n < sample_size:
            # for debugging
            if n % 290 == 0:
                print("working...index: ", n)
            grapple_point_index = n % num_grapple_points #which grapple point set are we adding to
            config = generate_sample(spec, grapple_point_index)
            if not is_step_collision(spec, config):
                new_node = GraphNode(spec, config)
                new_node = connect_node_to_neighbours(new_node, graph_nodes[grapple_point_index])
                graph_nodes[grapple_point_index].append(new_node)
                n += 1 
        
        end_node = None
        start_node = init_node
        for index in range(num_grapple_points):
            path = None
            if index == (num_grapple_points - 1):
                path = find_graph_path(spec, start_node, goal_node)
            else:
                end_node = find_closest_node_in_adjacent_set(spec, graph_nodes[index+1], index)
                end_node = connect_node_to_neighbours(end_node, graph_nodes[index])
                path = find_graph_path(spec, start_node, end_node)
            if path is None:
                break
            isPathFound = True
            path = interpolate_path(spec, path, index)
            path = detect_and_correct_violations(spec, path) #recursive call to generate_steps(...) is in here

            steps.extend(path)
            start_node = end_node

        if isPathFound is False:
            continue
    return steps

def main(arglist):
    input_file = arglist[0]
    output_file = arglist[1]

    spec = ProblemSpec(input_file)

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)

    num_grapple_points = spec.num_grapple_points
    if num_grapple_points == 1:
        graph_nodes = [[init_node, goal_node]]
    else:
        graph_nodes = [[] for i in range(num_grapple_points)]
        graph_nodes[0].append(init_node)
        graph_nodes[-1].append(goal_node)

    steps = []

    # Solution

    # Note on solution: if there are multiple grapple points, samples are collected separately for each grapple point
    # Then, the closest node to a grapple point in the sample for an ADJACENT grapple point is taken as the goal node.
    # 
    
    steps = generate_steps(spec, 5000*spec.num_grapple_points, graph_nodes, init_node, goal_node, spec.num_grapple_points)
            

    if len(arglist) > 1:
        write_robot_config_list_to_file(output_file, steps)

    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #
    v = Visualiser(spec, steps)


if __name__ == '__main__':
    main(sys.argv[1:])

