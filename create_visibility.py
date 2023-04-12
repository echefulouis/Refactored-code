import numpy as np
from numpy import *
from shapely.geometry import Point, Polygon
from collections import defaultdict
import os
import math
from helpers.graph import *
from helpers.geometry import *
import itertools
import pylab
from pylab import figure, show, rand

class Visibility:
    def __init__(self,obstacles,graph_points,env,obs1,obs2,obs3,radius=150):
        self.obstacles = obstacles
        self.radius = radius
        self.graph_points_list = graph_points
        self.visibility_polygons = defaultdict(lambda: defaultdict(list))
        self.visibility_polygons_area = defaultdict(lambda: defaultdict(float))
        self.max_sector_visibility_polygon_area = {}
        self.vertex_sequence=[]
        self.freecell = []
        self.ltable = {}
        self.lvispoly = {}
        self.env = [tuple(i) for i in env]
        self.obs1= [tuple(i) for i in obs1]
        self.obs2= [tuple(i) for i in obs2]
        self.obs3= [tuple(i) for i in obs3]
        self.maxc_x=200
        self.maxr_y=300
        self.directions = {31545: [], 45135: [], 135225: [], 225315: []}

    def polar_point(self,origin_point, angle, distance):
        return [origin_point.x + math.sin(math.radians(angle)) * distance,
                origin_point.y + math.cos(math.radians(angle)) * distance]

    def point_in_poly(self,x, y, poly):

        n = len(poly)
        inside = False

        p1x, p1y = poly[0]
        for i in range(n + 1):
            p2x, p2y = poly[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def midcellnumber(self,y, x):
        cell = floor(y) * self.maxc_x + floor(x)

        return int(cell)

    def insidenvironment(self, x, y):
        if self.point_in_poly(x, y, self.env):
            return True
        else:
            return False
        pass

    def insideobstacle(self, x, y):
        if self.point_in_poly(x, y, self.obs1) or self.point_in_poly(x, y, self.obs2) or self.point_in_poly(x, y, self.obs3):
            return True
        else:
            return False
        pass

    def sector(self, center, start_angle, end_angle, radius, steps=200):
        if start_angle > end_angle:
            start_angle = start_angle - 360
        else:
            pass
        step_angle_width = (end_angle - start_angle) / steps
        sector_width = (end_angle - start_angle)
        segment_vertices = []

        segment_vertices.append(self.polar_point(center, 0, 0))
        segment_vertices.append(self.polar_point(center, start_angle, radius))

        for z in range(1, steps):
            segment_vertices.append((self.polar_point(center, start_angle + z * step_angle_width, radius)))
        segment_vertices.append(self.polar_point(center, start_angle + sector_width, radius))
        segment_vertices.append(self.polar_point(center, 0, 0))
        return Polygon(segment_vertices)
        pass

    def genVisibMatrix(self,guardno):
        a = str(os.popen("./main gridnewenvironment2 gridnewguards " + str(guardno)).read())
        l = a.split()
        mat = []
        # print(a)
        for i in range(0, len(l) - 1, 2):
            mat.append([float(l[i]), float(l[i + 1])])
        return mat

    def calculate_visibility_polygons(self):
        for index, position in enumerate(self.graph_points_list):
            x, y = position
            if not self.insidenvironment(x, y) or self.insideobstacle(x, y):
                continue
            else:
                self.freecell.append(index)
                np.savetxt('gridnewguards', (x, y), fmt='%5.1f')
                ccellno = int(self.midcellnumber(y, x))
                sectorpolyd1 = self.sector(Point(x, y), 45, 135, self.radius)
                sectorpolyd2 = self.sector(Point(x, y), 315, 45, self.radius)
                sectorpolyd3 = self.sector(Point(x, y), 225, 315, self.radius)
                sectorpolyd4 = self.sector(Point(x, y), 135, 225, self.radius)
                polygon1 = self.genVisibMatrix(0)

                points = []
                for i in range(len(polygon1)):
                    points.append(Point(polygon1[i][0], polygon1[i][1]))
                polygon1 = Polygon([[p.x, p.y] for p in points])

                sectorviewd1 = polygon1.intersection(sectorpolyd1)
                sectorviewd2 = polygon1.intersection(sectorpolyd2)
                sectorviewd3 = polygon1.intersection(sectorpolyd3)
                sectorviewd4 = polygon1.intersection(sectorpolyd4)

                sectorcoordsd1 = list(sectorviewd1.exterior.coords)
                sectorcoordsd2 = list(sectorviewd2.exterior.coords)
                sectorcoordsd3 = list(sectorviewd3.exterior.coords)
                sectorcoordsd4 = list(sectorviewd4.exterior.coords)

                self.visibility_polygons[index][31545] = sectorcoordsd1
                self.visibility_polygons[index][45135] = sectorcoordsd2
                self.visibility_polygons[index][135225] = sectorcoordsd3
                self.visibility_polygons[index][225315] = sectorcoordsd4

                self.visibility_polygons_area[index][31545] = sectorviewd1.area
                self.visibility_polygons_area[index][45135] = sectorviewd2.area
                self.visibility_polygons_area[index][135225] = sectorviewd3.area
                self.visibility_polygons_area[index][225315] = sectorviewd4.area

                # fig = figure(figsize=(18, 16))
                # ax = fig.add_subplot(111, aspect='equal', xlabel="S", ylabel="t")

                # ax.add_patch(MPolygon(sectorcoordsd1, closed=True, fill=True, color='r', linewidth=0))
                # ax.add_patch(MPolygon(sectorcoordsd2, closed=True, fill=True, color='r', linewidth=0))
                # ax.add_patch(MPolygon(sectorcoordsd3, closed=True, fill=True, color='r', linewidth=0))
                # ax.add_patch(MPolygon(sectorcoordsd4, closed=True, fill=True, color='r', linewidth=0))
                #
                # ax.add_patch(MPolygon(LineList, closed=True, fill=False, color='black', label='line 1', linewidth=3))
                # ax.add_patch(MPolygon(hole1, closed=True, fill=True, color='black', label='line 1', linewidth=2))
                # ax.add_patch(MPolygon(hole2, closed=True, fill=True, color='black', label='line 1', linewidth=2))
                # ax.add_patch(MPolygon(hole3, closed=True, fill=True, color='black', label='line 1', linewidth=2))
                #
                # ax.add_patch(unitA)
                # ax.set_xlim(-2, 300)
                # ax.set_ylim(-2, 200)

                # timestr = time.strftime("%Y%m%d-%H%M%S")
                # pylab.savefig('output/'+timestr+'No_'+str(l)+".jpeg", bbox_inches='tight')
                # show()


    def calculate_vertexsquence_from_freespace(self):
        free_env_space1 = Polygon(self.env).difference(Polygon(self.obs1))
        free_env_space2 = free_env_space1.difference(Polygon(self.obs2))
        free_env_space = free_env_space2.difference(Polygon(self.obs3))

        free_space = free_env_space

        tmp_visibility_polygons = self.visibility_polygons.copy()

        directions_keys = list(self.directions.keys())
        count = 0
        max_value_count=0
        while not free_space.is_empty:
            count += 1
            max_visibility_area = 0
            max_vertex_index = 0
            max_visibilty_polygon = 0
            for i in tmp_visibility_polygons.keys():
                value_count = 0
                for value in tmp_visibility_polygons[i].values():
                    try:
                        max_sector = free_space.intersection(Polygon(value))
                    except Exception as e:
                        if "TopologyException" in str(e):
                            free_space = free_space.buffer(0.001).simplify(0.001)
                            max_sector = free_space.intersection(Polygon(value))
                    if (max_sector.area > max_visibility_area):
                        max_visibility_area = max_sector.area
                        max_visibilty_polygon = max_sector
                        max_visibilty_polygon_for_view = value
                        max_value_count = value_count
                        max_vertex_index = i
                    value_count += 1
            self.max_sector_visibility_polygon_area[max_vertex_index] = max_visibility_area
            self.vertex_sequence.append(max_vertex_index)
            tmp_visibility_polygons[max_vertex_index][directions_keys[max_value_count]] = []
            self.visibility_polygons[max_vertex_index][directions_keys[max_value_count]] = 0
            free_space = free_space.difference(max_visibilty_polygon)

        print(self.vertex_sequence)
