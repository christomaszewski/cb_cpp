import numpy as np
import shapely.geometry
import operator

from .base import ConstraintLayout
from .constraint import OpenConstraint, ClosedConstraint

class BoustrophedonPattern(ConstraintLayout):

	def __init__(self, sensor_radius, vehicle_radius, **unknown_options):
		# TODO Add orientation support
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius

	def layout_constraints(self, area, **unknown_options):
		# TODO needs to be able to lay out constraints in any direction
		x_min, y_min, x_max, y_max = area.bounds
		area_width = x_max - x_min
		if area_width < 2*self._vehicle_radius:
			print('Error: Cell width is smaller than diameter of vehicle')
			return None
		elif area_width < 2*self._sensor_radius:
			center_axis_x = x_min + area_width/2
			coords = [(center_axis_x, y_min+self._vehicle_radius), (center_axis_x, y_max-self._vehicle_radius)]
			return [OpenConstraint(coords)]		

		offset_polygon = area.polygon.buffer(-self._vehicle_radius, join_style=2)

		x_min, _, x_max, _ = offset_polygon.bounds
		_, y_min, _, y_max = area.bounds

		x_dist = x_max - x_min
		num_constraints = np.ceil(x_dist / (2*self._sensor_radius)).astype(int)
		transect_width = np.around(x_dist / num_constraints, decimals=5)

		current_x_pos = x_min
		constraints = []

		line_coords = [(current_x_pos, y_min), (current_x_pos, y_max)]
		sweep_line = shapely.geometry.LineString(line_coords)

		while current_x_pos <= x_max:
			if offset_polygon.intersects(sweep_line):
				intersection = offset_polygon.intersection(sweep_line)
				
				intersection_coords = list(intersection.coords)

				# may not need to sort these points anymore
				intersection_coords.sort(key=operator.itemgetter(1))

				constraints.append(OpenConstraint(intersection_coords))
				current_x_pos += transect_width
				
				line_coords = [(current_x_pos, y_min), (current_x_pos, y_max)]
				sweep_line = shapely.geometry.LineString(line_coords)

		return constraints

class HorizontalBoustrophedonPattern(ConstraintLayout):

	def __init__(self, sensor_radius, vehicle_radius, **unknown_options):
		# TODO Add orientation support
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius

	def layout_constraints(self, area, **unknown_options):
		# Horizonal Boustrophedon pattern, TODO: unify into universal boustrophedon patten
		x_min, y_min, x_max, y_max = area.bounds
		
		area_width = y_max - y_min

		if area_width < 2*self._vehicle_radius:
			print('Error: Cell width is smaller than diameter of vehicle')
			return None
		elif area_width < 2*self._sensor_radius:
			center_axis_y = y_min + area_width/2
			coords = [(x_min+self._vehicle_radius, center_axis_y), (x_max-self._vehicle_radius, center_axis_y)]
			return [OpenConstraint(coords)]		

		offset_polygon = area.polygon.buffer(-self._vehicle_radius, join_style=2)

		_, y_min, _, y_max = offset_polygon.bounds
		#_, y_min, _, y_max = area.bounds

		y_dist = area_width
		num_constraints = np.ceil(y_dist / (2*self._sensor_radius)).astype(int)
		transect_width = np.around(y_dist / num_constraints, decimals=5)

		current_y_pos = y_min
		constraints = []

		line_coords = [(x_min, current_y_pos), (x_max, current_y_pos)]
		sweep_line = shapely.geometry.LineString(line_coords)

		while current_y_pos <= y_max:
			if offset_polygon.intersects(sweep_line):
				intersection = offset_polygon.intersection(sweep_line)
				
				intersection_coords = list(intersection.coords)

				# may not need to sort these points anymore
				intersection_coords.sort(key=operator.itemgetter(0))

				constraints.append(OpenConstraint(intersection_coords))
				current_y_pos += transect_width
				
				line_coords = [(x_min, current_y_pos), (x_max, current_y_pos)]
				sweep_line = shapely.geometry.LineString(line_coords)

		return constraints


class SpiralPattern(ConstraintLayout):

	def __init__(self, sensor_radius, vehicle_radius, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius

	def layout_constraints(self, area, **unknown_options):
		offset_polygon = area.polygon.buffer(-self._vehicle_radius, join_style=2)
		constraints = []

		while offset_polygon.exterior:
			constraints.append(ClosedConstraint(offset_polygon.exterior.coords[:-1]))
			offset_polygon = offset_polygon.buffer(-2.*self._vehicle_radius, join_style=2)

		return constraints
