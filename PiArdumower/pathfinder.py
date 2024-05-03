import logging
logger = logging.getLogger(__name__)

import networkx as nx
from shapely.geometry import *
from shapely import affinity
from shapely.ops import *
import pandas as pd
from dataclasses import dataclass, field, asdict
#local imports
#from ..data.mapdata import current_map

@dataclass

class Perimeter:
    name: str = ''
    angle_offset: int = 0
    perimeter: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    perimeter_polygon: Polygon = Polygon()
    selected_perimeter: Polygon = Polygon()
    perimeter_for_plot: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    perimeter_points: MultiPoint = MultiPoint()
    search_wire: LineString = LineString()
    search_wire_points: MultiPoint = MultiPoint()
    gotopoints: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    gotopoint: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    mowpath: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    preview: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    obstacles: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    #obstacle_img: Image = field(default_factory = lambda: 
     #                           Image.open(os.path.dirname(__file__).replace('/backend/data', '/assets/icons/obstacle.png')))
   
    obstacle_img: pd.DataFrame = field(default_factory=lambda: pd.DataFrame())
    
    astar_graph: nx.Graph = nx.Graph()
    areatomow: float = 0
    distancetogo: float = 0
    map_crc: int = None
    current_perimeter_file: str = ''
    plotgotopoints: bool = False
    # Mow progress
    finished_distance = 0
    distance = 0
    distance_perc = 0
    finished_idx = 0
    idx = 0
    idx_perc = 0 
    # Progress bar
    calculating: bool = False
    calculated_progress: int = 0
    total_progress: int = 0
    task_progress: int = 0
    total_tasks: int = 0

current_map=Perimeter()
class PathFinder:
    angle: int = 0
    perimeter: Polygon = Polygon()
    perimeter_points: MultiPoint = MultiPoint()
    search_wire: LineString = LineString()
    search_wire_points: MultiPoint = MultiPoint()
    G: nx.Graph = nx.Graph()
    Gnew: nx.Graph = nx.Graph()

    def create(self) -> None:
        self.perimeter = current_map.perimeter_polygon.buffer(0.01, resolution=16, join_style=2, mitre_limit=1, single_sided=True)
        self.perimeter_points = current_map.perimeter_points
        self.search_wire = current_map.search_wire
        self.search_wire_points = current_map.search_wire_points
        self.G = current_map.astar_graph
        self.Gnew = current_map.astar_graph

    def check_direct_way(self, start, end) -> bool:
        way = LineString([start, end])
        if way.length <= 0.01:
            direct_way_possible = True
        else:
            direct_way_possible = way.within(self.perimeter)
        return direct_way_possible

    def add_edges(self, point: Point) -> None:
        nearest_point = nearest_points(point, self.perimeter)[1]
        #Check edges to perimeter and exclusions
        if self.check_direct_way(list(point.coords)[0], list(nearest_point.coords)[0]):
            direct_way = LineString((list(point.coords)[0], list(nearest_point.coords)[0]))
            self.Gnew.add_edge(list(direct_way.coords)[0], list(direct_way.coords)[1], weight=direct_way.length)
        for possible_point in self.perimeter_points.geoms:
            if self.check_direct_way(list(point.coords)[0], list(possible_point.coords)[0]):
                direct_way = LineString((list(point.coords)[0], list(possible_point.coords)[0]))
                self.Gnew.add_edge(list(direct_way.coords)[0], list(direct_way.coords)[1], weight=direct_way.length)
            if self.check_direct_way(list(nearest_point.coords)[0], list(possible_point.coords)[0]):
                direct_way = LineString((list(nearest_point.coords)[0], list(possible_point.coords)[0]))
                self.Gnew.add_edge(list(direct_way.coords)[0], list(direct_way.coords)[1], weight=direct_way.length)
        #Check edges to search wire
        if not self.search_wire.is_empty:
            nearest_point = nearest_points(point, self.search_wire)[1]
            if self.check_direct_way(list(point.coords)[0], list(nearest_point.coords)[0]):
                direct_way = LineString((list(point.coords)[0], list(nearest_point.coords)[0]))
                self.Gnew.add_edge(list(direct_way.coords)[0], list(direct_way.coords)[1], weight=direct_way.length)
            for possible_point in self.search_wire_points.geoms:
                if self.check_direct_way(list(point.coords)[0], list(possible_point.coords)[0]):
                    direct_way = LineString((list(point.coords)[0], list(possible_point.coords)[0]))
                    self.Gnew.add_edge(list(direct_way.coords)[0], list(direct_way.coords)[1], weight=direct_way.length)
                if self.check_direct_way(list(nearest_point.coords)[0], list(possible_point.coords)[0]):
                    direct_way = LineString((list(nearest_point.coords)[0], list(possible_point.coords)[0]))
                    self.Gnew.add_edge(list(direct_way.coords)[0], list(direct_way.coords)[1], weight=direct_way.length)

    def find_way(self, start: list(), goal: list()) -> list:
        start = affinity.rotate(Point(start), self.angle, origin=(0, 0))
        goal = affinity.rotate(Point(goal), self.angle, origin=(0, 0))
        logger.debug('Pathfinder start: '+str(list(start.coords)) +' goal: '+str(list(goal.coords)))
        self.add_edges(start)
        self.add_edges(goal)
        try:
            astar_path = nx.astar_path(self.Gnew, list(start.coords)[0], list(goal.coords)[0], heuristic=None, weight='weight') 
            logger.debug('Pathfinder found a way: '+str(astar_path))
            path = LineString(astar_path)
            path = affinity.rotate(path, -self.angle, origin=(0, 0))
            del astar_path[0] #remove first point it is given by legacy route 
            return list(path.coords)
        except Exception as e:
            logger.warning('Pathfinder could not find a way. Action aborted')
            logger.debug(str(e))
            return list()

pathfinder = PathFinder()

