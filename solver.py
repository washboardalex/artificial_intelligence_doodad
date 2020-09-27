import sys
import random
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


def find_graph_path(spec, init_node):
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

        if test_config_equality(current.config, spec.goal, spec):
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


def generate_sample(spec):
    """
    Generates a sample robot config taking a problem spec as input.
    """

    new_angles = generate_angles(spec.num_segments) # num_segments == num_angles 
    new_lengths = generate_lengths(spec.min_lengths, spec.max_lengths)
    
    # for now only implementing grapple points == 1
    if spec.num_grapple_points == 1:
        #possible bug if EE2 is the one grappled
        robot_config = make_robot_config_from_ee1(spec.grapple_points[0][0], spec.grapple_points[0][1], new_angles, new_lengths)
    else:
        is_ee1_grappled = bool(random.getrandbits(1))
        sample_grapple_point_index = random.randrange(0, len(spec.grapple_points))
        new_x = spec.grapple_points[sample_grapple_point_index][0]
        new_y = spec.grapple_points[sample_grapple_point_index][1]
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
def interpolate_path(spec, start, end): 
    primitive_step = spec.PRIMITIVE_STEP #remove this once you update the spec to original value
    interpolated_path = [start]
    
    #designate an index
    is_in_tolerance = test_config_distance(start, end, spec)
    # index = 0
    while not is_in_tolerance:
        # index += 1
        # if index % 290 == 0:
        #     print("interpolating...index: ", index)
        new_angles = generate_new_vals(interpolated_path[-1].ee1_angles, end.ee1_angles, primitive_step)
        new_lengths = generate_new_vals(interpolated_path[-1].lengths, end.lengths, primitive_step)
        new_config = make_robot_config_from_ee1(spec.grapple_points[0][0], spec.grapple_points[0][1], new_angles, new_lengths, ee1_grappled = True)
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
            violations.append((step1, step2))
    
    return violations

def main(arglist):
    input_file = arglist[0]
    output_file = arglist[1]

    spec = ProblemSpec(input_file)

    init_node = GraphNode(spec, spec.initial)
    goal_node = GraphNode(spec, spec.goal)
    graph_nodes = [init_node, goal_node]

    steps = []

    # Solution

    # Check if the beginning and ending lengths are the same. If they are, then do one algorithm, if not do another thing

    isPathFound = False
    while isPathFound is False:
        # print("its false!")
        n = 0
        sample_size = 5000 # + ((spec.num_grapple_points - 1) * 1000)
        print("sample size: ", sample_size)
        while n < sample_size:
            # for debugging
            if n % 290 == 0:
                print("working...index: ", n)
            config = generate_sample(spec)
            # check if the configuration is a collision
            # if not then append.
            if not is_step_collision(spec, config):
                new_node = GraphNode(spec, config)
                # if some distance condition is satisfied, based on config, add neighbours to newnode
                for node in graph_nodes:
                    node_angles, new_node_angles, node_lengths, new_node_lengths = node.config.ee1_angles, new_node.config.ee1_angles, node.config.lengths, new_node.config.lengths
                    if is_connect_angles(node_angles, new_node_angles) and is_connect_lengths(node_lengths, new_node_lengths):
                        GraphNode.add_connection(node, new_node)
                graph_nodes.append(new_node)
                n += 1
        path = find_graph_path(spec, init_node)
        if path is not None:
            isPathFound = True
            steps = path
            steps.append(spec.goal)
        else:
            continue

        # First, do a round of simple linear interpolation in C-Space
        num_steps = len(steps)
        interpolated_path = []
        for i in range(num_steps - 1):
            new_path_segment = interpolate_path(spec, steps[i], steps[i+1])
            interpolated_path.extend(new_path_segment)
        interpolated_path.append(steps[-1])
        steps = interpolated_path

        # Then, wherever collisions exist, recursively construct a new graph in violation space until no violations
        # We'll set a limit to recursive depth in case an impossible path has been chosen.
        # Currently simply resamples the whole graph which is wasteful of resources
        num_steps = len(steps)
        violations = search_steps_violations(spec, steps)
        if len(violations) > 0:
            print("There are some violations")
            print(len(violations))
            print("The violations are : ")
            for violation in violations:
                for config in violation:
                    print(config.points)
            

    if len(arglist) > 1:
        write_robot_config_list_to_file(output_file, steps)

    #
    # You may uncomment this line to launch visualiser once a solution has been found. This may be useful for debugging.
    # *** Make sure this line is commented out when you submit to Gradescope ***
    #
    v = Visualiser(spec, steps)


if __name__ == '__main__':
    main(sys.argv[1:])

