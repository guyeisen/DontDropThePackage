import itertools
import math
import threading
from typing import List

import networkx as nx
import matplotlib.pyplot as plt
from CGALPY.Ker import squared_radius
from discopygal.bindings import Segment_2, Point_2, Point_d

from discopygal.solvers import Robot, RobotDisc, RobotPolygon, RobotRod
from discopygal.solvers import Obstacle, ObstacleDisc, ObstaclePolygon, Scene
from discopygal.solvers import PathPoint, Path, PathCollection, Solver


from discopygal.solvers.metrics import Metric, Metric_Euclidean
from discopygal.solvers.nearest_neighbors import NearestNeighbors, NearestNeighbors_sklearn
from discopygal.solvers.samplers import Sampler, Sampler_Uniform
from discopygal.geometry_utils import conversions
import collision_detection

#from smooth_path import get_circle
from error_handling import PathNotFoundException
from smooth_path import get_circle, get_angle

lock = threading.Lock()
NUM_OF_LANDMARKS = 100
NEAEREST_NEIGHBOUR = 15

class PointForOptimization:
    """ (connected_to)-------(p)"""
    def __init__(self, point: Point_d, connected_to: Point_d):
        self.point = point
        self.connected_to = connected_to

    def __str__(self):
        return f"main:({self.point})--->({self.connected_to})"
    def __eq__(self, other):
        return self.point==other.point and self.connected_to==other.connected_to
    def __hash__(self):
        return hash(self.__str__())

class OptimizedGraph(nx.Graph):
    def __init__(self):
        super(OptimizedGraph, self).__init__()
    def add_edge(self, u_of_edge:PointForOptimization ,v_of_edge:PointForOptimization, **attr):
        """Overriden method for adding edge. keeping that a new edge won't insert already exists nodes."""
        u, v = u_of_edge, v_of_edge
        # if u.point == v.point and u.connected_to == v.connected_to :
        #     return #means this is not an edge, this is a point in graph - nothing to do
        super().add_edge(u, v, **attr)

class PRM(Solver):
    """
    The basic implementation of a Probabilistic Road Map (PRM) solver.
    Supports multi-robot motion planning, though might be inefficient for more than
    two-three robots.
    :param num_landmarks: number of landmarks to sample
    :type num_landmarks: int
    :param k: number of nearest neighbors to connect
    :type k: int
    :param nearest_neighbors: a nearest neighbors algorithm. if None then use sklearn implementation
    :type nearest_neighbors: class:'NearestNeighbors' or None
    :param metric: a metric for weighing edges, can be different then the nearest_neighbors metric!
        If None then use euclidean metric
    :type metric: class:'Metric' or None
    :param sampler: sampling algorithm/method. if None then use uniform sampling
    :type sampler: class:'Sampler'
    """

    def __init__(self, num_landmarks, k, nearest_neighbors=None, metric=None, sampler=None):
        super().__init__()
        self.num_landmarks = num_landmarks
        self.k = k

        self.nearest_neighbors: NearestNeighbors = nearest_neighbors
        if self.nearest_neighbors is None:
            self.nearest_neighbors = NearestNeighbors_sklearn()

        self.metric: Metric = metric
        if self.metric is None:
            self.metric = Metric_Euclidean

        self.sampler: Sampler = sampler
        if self.sampler is None:
            self.sampler = Sampler_Uniform()

        self.roadmap = None
        self.roadmap_optimized = None
        self.collision_detection = {}
        self.start = None
        self.end = None

    @staticmethod
    def get_arguments():
        """
        Return a list of arguments and their description, defaults and types.
        Can be used by a GUI to generate fields dynamically.
        Should be overridded by solvers.
        :return: arguments dict
        :rtype: dict
        """
        return {
            'num_landmarks': ('Number of Landmarks:', NUM_OF_LANDMARKS, int),
            'k': ('K for nearest neighbors:', NEAEREST_NEIGHBOUR, int),
        }

    @staticmethod
    def from_arguments(d):
        """
        Get a dictionary of arguments and return a solver.
        Should be overridded by solvers.
        :param d: arguments dict
        :type d: dict
        """
        return PRM(d['num_landmarks'], d['k'], None, None, None)

    def get_graph(self):
        """
        Return a graph (if applicable).
        Can be overridded by solvers.
        :return: graph whose vertices are Point_2 or Point_d
        :rtype: class:'networkx.Graph' or None
        """
        return self.roadmap

    def collision_free(self, p, q):
        """
        Get two points in the configuration space and decide if they can be connected
        """
        p_list = conversions.Point_d_to_Point_2_list(p)
        q_list = conversions.Point_d_to_Point_2_list(q)

        # Check validity of each edge seperately
        for i, robot in enumerate(self.scene.robots):
            edge = Segment_2(p_list[i], q_list[i])
            if not self.collision_detection[robot].is_edge_valid(edge):
                return False

        # Check validity of coordinated robot motion
        for i, robot1 in enumerate(self.scene.robots):
            for j, robot2 in enumerate(self.scene.robots):
                if j <= i:
                    continue
                edge1 = Segment_2(p_list[i], q_list[i])
                edge2 = Segment_2(p_list[j], q_list[j])
                if collision_detection.collide_two_robots(robot1, edge1, robot2, edge2):
                    return False

        return True

    def sample_free(self):
        """
        Sample a free random point
        """
        p_rand = []
        for robot in self.scene.robots:
            sample = self.sampler.sample()
            while not self.collision_detection[robot].is_point_valid(sample):
                sample = self.sampler.sample()
            p_rand.append(sample)
        p_rand = conversions.Point_2_list_to_Point_d(p_rand)
        return p_rand

    def load_scene(self, scene: Scene):
        """
        Load a scene into the solver.
        Also build the roadmap.
        :param scene: scene to load
        :type scene: class:'Scene'
        """
        super().load_scene(scene)
        self.sampler.set_scene(scene)

        # Build collision detection for each robot
        for robot in scene.robots:
            self.collision_detection[robot] = collision_detection.ObjectCollisionDetection(scene.obstacles, robot)

        ################
        # Build the PRM
        ################
        self.roadmap = nx.Graph()
        self.roadmap_optimized = OptimizedGraph()

        # Add start & end points
        self.start = conversions.Point_2_list_to_Point_d([robot.start for robot in scene.robots])
        self.end = conversions.Point_2_list_to_Point_d([robot.end for robot in scene.robots])

        self.roadmap.add_node(self.start)
        self.roadmap.add_node(self.end)

        self.start_opt = PointForOptimization(self.start, self.start)
        self.end_opt = PointForOptimization(self.end, self.end)
        self.roadmap_optimized.add_node(self.start_opt)
        self.roadmap_optimized.add_node(self.end_opt)
        # Add valid points
        for i in range(self.num_landmarks):
            p_rand = self.sample_free()
            self.roadmap.add_node(p_rand)
            if i % 100 == 0 and self.verbose:
                print('added', i, 'landmarks in PRM', file=self.writer)

        self.nearest_neighbors.fit(list(self.roadmap.nodes))

        dic_optPoint_to_optPoint_of_neighbor = {}
        # Connect all points to their k nearest neighbors
        for cnt, point in enumerate(self.roadmap.nodes):
            neighbors = self.nearest_neighbors.k_nearest(point, self.k + 1)
            mini_cluster = []
            for neighbor in neighbors:
                if self.collision_free(neighbor, point):
                    self.roadmap.add_edge(point, neighbor, weight=self.metric.dist(point, neighbor).to_double())
                    opt_p = PointForOptimization(point,neighbor)
                    opt_neighbor = PointForOptimization(neighbor,point)
                    dic_optPoint_to_optPoint_of_neighbor[opt_neighbor] = opt_p
                    dic_optPoint_to_optPoint_of_neighbor[opt_p] = opt_neighbor
                    self.roadmap_optimized.add_node(opt_p)
                    self.roadmap_optimized.add_node(opt_neighbor)
                    mini_cluster.append(opt_p)


            # create the "dummy" edges
            for p1,p2 in itertools.combinations(mini_cluster,2):
                self.add_optimizing_edge(p1,p2)

            if cnt % 100 == 0 and self.verbose:
                print('connected', cnt, 'landmarks to their nearest neighbors', file=self.writer)

        #For the rest of the edges in self.roadmap.optimized, I have to go over them again..

        for i,opt_point in enumerate(list(self.roadmap_optimized.nodes)):
            if opt_point in dic_optPoint_to_optPoint_of_neighbor:
                neigh = dic_optPoint_to_optPoint_of_neighbor[opt_point]
                if opt_point != neigh:
                    self.roadmap_optimized.add_edge(opt_point, neigh, weight=self.metric.dist(opt_point.point, neigh.point).to_double())
                    dic_optPoint_to_optPoint_of_neighbor.pop(opt_point, False)
                    dic_optPoint_to_optPoint_of_neighbor.pop(neigh, False)
        print("HEY!")


    def add_optimizing_edge(self, p1:PointForOptimization, p2:PointForOptimization):
        """wrapper to adding edge with weight as a function. default is the function 1/(1+R^2)
            where R is the circle raduis that those segments create"""
        v0 = Point_2(p1.connected_to[2 * 0], p1.connected_to[2 * 0 + 1])
        v1 = Point_2(p1.point[2 * 0], p1.point[2 * 0 + 1])
        v2 = Point_2(p2.connected_to[2 * 0], p2.connected_to[2 * 0 + 1])
        if v0 != v1 and v1 != v2:
            c = get_circle(v0, v1, v2)
            r_squre= c.squared_radius().to_double()
            r_squre = math.sqrt(r_squre)
            angle = get_angle(v0,v1,v2)
            if angle < 0:
                print("HAVE NEGATIVE ANGLE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            factor = 1+angle
            func1 = (angle**2)/(2*math.pi) - angle + math.pi/2
            func = func1 if angle >2.5 else 5/(1+angle)
            self.roadmap_optimized.add_edge(p1, p2,weight=func1)
        else:
            self.roadmap_optimized.add_edge(p1,p2,weight=0)
    def _print_angles(self,points:List[PathPoint]):
        angles = []
        for i in range(len(points)-2):
            angles.append(get_angle(points[i].location,points[i+1].location,points[i+2].location))
        print([math.degrees(angle) for angle in angles])
        print(f"SUM: {sum([(1/(1+angle)) for angle in angles])}")

    def solve(self):
        """
        Based on the start and end locations of each robot, solve the scene
        (i.e. return paths for all the robots)
        :return: path collection of motion planning
        :rtype: class:'PathCollection'
        """

        if not nx.algorithms.has_path(self.roadmap, self.start, self.end):
            if self.verbose:
                print('No path found...', file=self.writer)
            return PathCollection(), PathCollection()
        if not nx.algorithms.has_path(self.roadmap_optimized, self.start_opt, self.end_opt):
            print("MY BUG")
            print('No path found...', file=self.writer)
            return PathCollection(), PathCollection()
        print("1")
        # Convert from a sequence of Point_d points to PathCollection
        try:
            tensor_path = nx.algorithms.shortest_path(self.roadmap, self.start, self.end, weight='weight')
            tensor_path_optimized = nx.algorithms.shortest_path(self.roadmap_optimized,self.start_opt, self.end_opt ,weight='weight' )
        except Exception as e:
            print("Exception occured during shortest path calc")
            print(repr(e))
            raise PathNotFoundException
        path_collection = PathCollection()
        reset = {}
        path_collection_optimized = PathCollection(paths=reset) #PathCollection is defined with default argument set to {}! THAT MEAN ALL PATHCOLLECTION INSTANCES WILL SHARE THE SAME DIC!
        for i, robot in enumerate(self.scene.robots):
            points = []
            points_optimized = []
            k = 0
            for point in tensor_path:
                points.append(PathPoint(Point_2(point[2 * i], point[2 * i + 1])))
            print("SHORTEST:")
            self._print_angles(points)
            path = Path(points)
            in_path_dic={}
            for opt_point in tensor_path_optimized:
                if opt_point.point not in in_path_dic:
                    points_optimized.append(PathPoint(Point_2(opt_point.point[2*i],opt_point.point[2*i+1])))
                    in_path_dic[opt_point.point] = True
            print("OPTIMIZED:")
            self._print_angles(points_optimized)
            path_optimized = Path(points_optimized)
            path_collection.add_robot_path(robot, path)
            path_collection_optimized.add_robot_path(robot,path_optimized)
        if self.verbose:
            print('Successfully found a path!', file=self.writer)

        return path_collection, path_collection_optimized