import numpy as np
import shapely.geometry
import operator

from .base import ConstraintLayout
from .constraint import OpenConstraint, ClosedConstraint

class BoustrophedonPattern(ConstraintLayout):
	""" Deprecated: Use OrientedBoustrophedonPattern instead """

	def __init__(self, vehicle_radius, sensor_radius, **unknown_options):
		# TODO Add orientation support
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius

	def layout_constraints(self, area, **unknown_options):
		# TODO needs to be able to lay out constraints in any direction
		x_min, y_min, x_max, y_max = area.bounds
		area_width = x_max - x_min
		print(f"area width: {area_width}")
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
	""" Deprecated: Use OrientedBoustrophedonPattern Instead """

	def __init__(self, vehicle_radius, sensor_radius, **unknown_options):
		# TODO Add orientation support
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius

	def layout_constraints(self, area, **unknown_options):
		# Horizonal Boustrophedon pattern, TODO: unify into universal boustrophedon patten
		x_min, y_min, x_max, y_max = area.bounds
		
		area_width = y_max - y_min
		print(f"area width: {area_width}")

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

class OrientedBoustrophedonPattern(ConstraintLayout):

	def __init__(self, vehicle_radius, sensor_radius, sweep_direction, boundary_offset=None, **unknown_options):
		# TODO Add orientation support
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius
		self._sweep_direction  = np.array(sweep_direction) / np.linalg.norm(np.array(sweep_direction))
		if boundary_offset:
			if boundary_offset >= self._vehicle_radius:
				self._boundary_offset = boundary_offset
			else:
				print(f"Warning: Desired boundary offset {boundary_offset} is less than vehicle radius {vehicle_radius}. Setting offset to vehicle radius.")
				self._boundary_offset = vehicle_radius
		else:
			print('No boundary offset specified, setting to max of vehicle and sensor radii.')
			self._boundary_offset = max(sensor_radius, vehicle_radius)

	@classmethod
	def from_transect_orientation(cls, sensor_radius, vehicle_radius, transect_orientation, **other_options):
		sweep_direction = (-transect_orientation[1], transect_orientation[0])

		return cls(sensor_radius, vehicle_radius, sweep_direction, **other_options)

	def layout_constraints(self, area, compute_offset=True, **unknown_options):
		offset = self._boundary_offset

		if compute_offset:
			# Compute true max (while still ensuring coverage) boundary offset from corners of area
			# based on interior angles of polygon. No less than vehicle radius
			min_angle = min(area.interior_angles.values())
			max_offset = self._boundary_offset * np.sin(np.radians(min_angle/2.))

			offset = max(self._vehicle_radius, max_offset)

		# Compute direction vector of sweep line from perpendicular unit sweep direction vector
		sweep_line_direction = np.array([-self._sweep_direction[1], self._sweep_direction[0]])

		# Project area vertices onto sweep line vector to determine required extents of sweep line
		area_verts_scalar_proj = [np.dot(np.array(vert), sweep_line_direction) for vert in area.vertices]
		sweep_line_min = min(area_verts_scalar_proj) * sweep_line_direction
		sweep_line_max = max(area_verts_scalar_proj) * sweep_line_direction

		# Offset the area according to offset
		offset_area = area.polygon.buffer(-offset, join_style=2)

		# Project offset area vertices onto sweep direction vector to get coverage area width and sweep line start point
		offset_verts = list(offset_area.exterior.coords)[:-1]
		offset_verts_scalar_proj = [np.dot(np.array(vert), self._sweep_direction) for vert in offset_verts]
		offset_area_min = min(offset_verts_scalar_proj)
		start_point_index = np.argmin(offset_verts_scalar_proj)
		offset_area_max = max(offset_verts_scalar_proj)
		end_point_index = np.argmax(offset_verts_scalar_proj)
		offset_area_width = offset_area_max - offset_area_min

		""" 
		Because we are using offset width here, these checks wont work as intended for all cases
		e.g. if offset_area_width is less than 2* vehicle_radius, the vehicle can still fit in 
		the are because we've already offset the polygon by a minimum of the vehicle radius

		if offset_area_width < 2*self._vehicle_radius:
			print('Error: Cell width is smaller than diameter of vehicle')
			return None
		elif area_width < 2*self._sensor_radius:
			center_axis_y = y_min + area_width/2
			coords = [(x_min+self._vehicle_radius, center_axis_y), (x_max-self._vehicle_radius, center_axis_y)]
			return [OpenConstraint(coords)]

		"""

		# Instantiate array to hold constraints
		constraints = []

		# Compute the number of cells in the coverage area (regions between constraints)
		num_cells = np.ceil(offset_area_width / (2*self._sensor_radius)).astype(int)
		transect_width = np.around(offset_area_width / num_cells, decimals=5)

		sweep_line_coords = np.array([sweep_line_min, sweep_line_max])
		current_sweep_pos = offset_area_min
		sweep_line_coords += (current_sweep_pos * self._sweep_direction)

		line_coords = [tuple(pt) for pt in sweep_line_coords]
		sweep_line = shapely.geometry.LineString(line_coords)

		# Define delta to increment/decrement sweep line position by when searching
		# for intersections at edges of coverage area
		delta = 0.000001

		#while current_sweep_pos <= offset_area_max:
		# Want to have one more constraint than cells
		while len(constraints) <= num_cells:
			if offset_area.intersects(sweep_line):
				intersection = offset_area.intersection(sweep_line)
				intersection_coords = list(intersection.coords)

				# Sort intersection coordinates by distance along sweep_line - avoids issues with arbitrary ordering
				intersection_coords.sort(key=lambda coords: np.dot(np.array(coords), sweep_line_direction))

				if len(intersection_coords) > 1:
					constraints.append(OpenConstraint(intersection_coords))
				else:
					# Intersection occurs at a single point so we find the side of the
					# offset polygon that is most aligned with the transect orientation, i.e.  
					# least aligned with sweep direction, and use that as the first constraint
					corner = offset_verts[start_point_index]
					pt1 = offset_verts[(start_point_index-1)%len(offset_verts)]
					pt2 = offset_verts[(start_point_index+1)%len(offset_verts)]
					side_vec1 = np.array(pt1) - np.array(corner)
					side_vec1 /= np.linalg.norm(side_vec1)
					side_vec2 = np.array(pt2) - np.array(corner)
					side_vec2 /= np.linalg.norm(side_vec2)
					side_proj1 = np.dot(sweep_line_direction, side_vec1)
					side_proj2 = np.dot(sweep_line_direction, side_vec2)

					if abs(side_proj1) > abs(side_proj2):
						if side_proj1 < 0:
							constraints.append(OpenConstraint([pt1, corner]))
						else:
							constraints.append(OpenConstraint([corner, pt1]))
					else:
						if side_proj2 < 0:
							constraints.append(OpenConstraint([pt2, corner]))
						else:
							constraints.append(OpenConstraint([corner, pt2]))

				# Advance sweep line
				current_sweep_pos += transect_width
				sweep_line_coords += (transect_width * self._sweep_direction)
				
				line_coords = [tuple(pt) for pt in sweep_line_coords]
				sweep_line = shapely.geometry.LineString(line_coords)

			# Probably don't need this now since we compute the first and last constraints
			elif len(constraints) == 0: 
				print(f"Sweepline does not intersect polygon, advancing line by {delta}")
				# Line doesn't interesect polygon and no constraints have been found yet
				# Advancing slightly and trying again
				current_sweep_pos += delta
				sweep_line_coords += (delta * self._sweep_direction)
				line_coords = [tuple(pt) for pt in sweep_line_coords]
				sweep_line = shapely.geometry.LineString(line_coords)
			else:
				print(f"Sweepline does not intersect polygon, retreating line by {delta}")
				# Line doesn't intersect polygon and we've previously found intersections
				# Probably advanced beyond edge of polygon, retreat sweep line and try again
				current_sweep_pos -= delta
				sweep_line_coords -= (delta * self._sweep_direction)
				line_coords = [tuple(pt) for pt in sweep_line_coords]
				sweep_line = shapely.geometry.LineString(line_coords)
			
		return constraints

class SpiralPattern(ConstraintLayout):

	def __init__(self, vehicle_radius, sensor_radius, boundary_offset=None, **unknown_options):
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius
		if boundary_offset:
			if boundary_offset >= self._vehicle_radius:
				self._boundary_offset = boundary_offset
			else:
				print(f"Warning: Desired boundary offset {boundary_offset} is less than vehicle radius {vehicle_radius}. Setting offset to vehicle radius.")
				self._boundary_offset = vehicle_radius
		else:
			print('No boundary offset specified, setting to max of vehicle and sensor radii.')
			self._boundary_offset = max(sensor_radius, vehicle_radius)

	def layout_constraints(self, area, compute_offset=True, **unknown_options):
		offset = self._boundary_offset

		if compute_offset:
			# Compute true max (while still ensuring coverage) boundary offset from corners of area
			# based on interior angles of polygon. No less than vehicle radius
			min_angle = min(area.interior_angles.values())
			max_offset = self._boundary_offset * np.sin(np.radians(min_angle/2.))

			offset = max(self._vehicle_radius, max_offset)

		offset_polygon = area.polygon.buffer(-offset, join_style=2)
		constraints = []

		while offset_polygon.exterior:
			# Create closed constraint from coords of current offset polygon
			constraints.append(ClosedConstraint(offset_polygon.exterior.coords[:-1]))
			
			# Compute the max offset for the next polygon that still ensures complete coverage
			min_angle = min(area.interior_angles.values())
			max_offset = 2 * self._sensor_radius * np.sin(np.radians(min_angle/2.))			
			offset = max(self._vehicle_radius, max_offset)

			offset_polygon = offset_polygon.buffer(-offset, join_style=2)

		return constraints

class StreamlinePattern(ConstraintLayout):

	def __init__(self, vehicle_radius, sensor_radius, boundary_offset=None, **unknown_options):
		# TODO Add orientation support
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius
		if boundary_offset:
			if boundary_offset >= self._vehicle_radius:
				self._boundary_offset = boundary_offset
			else:
				print(f"Warning: Desired boundary offset {boundary_offset} is less than vehicle radius {vehicle_radius}. Setting offset to vehicle radius.")
				self._boundary_offset = vehicle_radius
		else:
			print('No boundary offset specified, setting to max of vehicle and sensor radii.')
			self._boundary_offset = max(sensor_radius, vehicle_radius)

	def layout_constraints(self, area, compute_offset=True, bias=None, **unknown_options):
		offset = self._boundary_offset

		if compute_offset:
			# Compute true max (while still ensuring coverage) boundary offset from corners of area
			# based on interior angles of polygon. No less than vehicle radius
			min_angle = min(area.interior_angles.values())
			max_offset = self._boundary_offset * np.sin(np.radians(min_angle/2.))

			offset = max(self._vehicle_radius, max_offset)

		# # Compute direction vector of sweep line from perpendicular unit sweep direction vector
		# sweep_line_direction = np.array([-self._sweep_direction[1], self._sweep_direction[0]])

		# # Project area vertices onto sweep line vector to determine required extents of sweep line
		# area_verts_scalar_proj = [np.dot(np.array(vert), sweep_line_direction) for vert in area.vertices]
		# sweep_line_min = min(area_verts_scalar_proj) * sweep_line_direction
		# sweep_line_max = max(area_verts_scalar_proj) * sweep_line_direction

		# # Offset the area according to offset
		offset_area = area.polygon.buffer(-offset, join_style=2)

		mid_idx = int(len(offset_area.exterior.coords)/2)
		bank_1 = [np.array(pt) for pt in offset_area.exterior.coords[:mid_idx]]
		bank_2 = [np.array(pt) for pt in reversed(offset_area.exterior.coords[mid_idx:-1])]
		print(len(area.polygon.exterior.coords), len(offset_area.exterior.coords), len(bank_1), len(bank_2))
		# print(len(offset_area.exterior.coords), mid_idx)
		# print(offset_area.exterior.coords[:3], offset_area.exterior.coords[mid_idx], offset_area.exterior.coords[-3:])
		# print(bank_1[-3:], bank_2[-3:], len(bank_1), len(bank_2))

		cross_section_lengths = [np.linalg.norm(pt2 - pt1) for pt1, pt2 in zip(bank_1, bank_2)]

		max_width = max(cross_section_lengths)
		min_width = min(cross_section_lengths)

		num_transects = int(np.ceil(max_width/(2*self._sensor_radius) - 1))
		print(f"Offset: {offset}")
		print(f"Max cross section width: {max_width}, Num Transects: {num_transects+2}")
		print(f"Min cross section width: {min(cross_section_lengths)}")
		print(offset_area.exterior.coords[:3], offset_area.exterior.coords[-3:])
		print(bank_1[:3], bank_1[-3:])
		print(bank_2[:3], bank_2[-3:])

		num_full_transects = int(min_width/(2*self._sensor_radius))

		transect_coords = [[] for i in range(num_transects+2)]
		for cross_section in zip(bank_1, bank_2):
			cross_vec = cross_section[1]-cross_section[0]
			length = np.linalg.norm(cross_vec)
			direction = cross_vec / length

			if bias == 'centerline':
				# Bias towards centerline
				transect_width = self._sensor_radius * 2.
				half_num_full_transects = int(length / (2*transect_width))
				max_full_transects = int(length / transect_width)

				print(f"Cross Section width: {length}, Max Transect width: {transect_width}, Max Full Transects: {max_full_transects}, {half_num_full_transects}")
				last_idx = 0
				for i in range(half_num_full_transects+1):
					transect_coords[i].append(tuple(cross_section[1] - i*transect_width*direction))
					last_idx = i

				print(f"Added {last_idx+1} transects before centerline")
				num_remaining_transects = num_transects - max_full_transects
				print(f"Remaining Transects: {num_remaining_transects}")

				last_coord = np.array(transect_coords[last_idx][-1])
				# Add some centerline transects
				for i in range(int(np.ceil(num_remaining_transects/2.))):
					print('Adding centerline transect')
					last_idx += 1
					transect_coords[last_idx].append(tuple(cross_section[1] - length / 2. * direction))
					last_coord = np.array(transect_coords[last_idx][-1])

				print(f"Adding {half_num_full_transects} transects after centerline")
				for i in range(half_num_full_transects):
					last_idx += 1
					transect_coords[last_idx].append(tuple(cross_section[0] + (half_num_full_transects - i)*transect_width*direction))

				# Collapse remaining transects to inner bank
				while last_idx < num_transects + 1:
					print('Adding bank transect')
					last_idx += 1
					transect_coords[last_idx].append(tuple(cross_section[0]))

			# if bank_bias:
			# 	# Bias towards centerline, then towards bank
			# 	transect_width = self._sensor_radius * 2.
			# 	half_num_full_transects = int(length / (2*transect_width))
			# 	max_full_transects = int(length / transect_width)

			# 	print(f"Cross Section width: {length}, Max Transect width: {transect_width}, Max Full Transects: {max_full_transects}, {half_num_full_transects}")
			# 	last_idx = 0
			# 	for i in range(half_num_full_transects+1):
			# 		transect_coords[i].append(tuple(cross_section[1] - i*transect_width*direction))
			# 		last_idx = i

			# 	print(f"Added {last_idx+1} transects before centerline")
			# 	num_remaining_transects = num_transects - max_full_transects
			# 	print(f"Remaining Transects: {num_remaining_transects}")

			# 	last_coord = np.array(transect_coords[last_idx][-1])
			# 	# Add some centerline transects
			# 	for i in range(int(np.ceil(num_remaining_transects/2.))):
			# 		print('Adding centerline transect')
			# 		last_idx += 1
			# 		transect_coords[last_idx].append(tuple(cross_section[1] - length / 2. * direction))
			# 		last_coord = np.array(transect_coords[last_idx][-1])

			# 	print(f"Adding {half_num_full_transects} transects after centerline")
			# 	for i in range(1,half_num_full_transects+1):
			# 		last_idx += 1
			# 		transect_coords[last_idx].append(tuple(last_coord - i*transect_width*direction))

			# 	# Collapse remaining transects to inner bank
			# 	while last_idx < num_transects + 1:
			# 		print('Adding bank transect')
			# 		last_idx += 1
			# 		transect_coords[last_idx].append(tuple(cross_section[0]))

			elif bias == 'inner_bank':
				# Bias towards inner bank and collapse redundant transects
				transect_width = self._sensor_radius * 2.
				max_full_transects = int(length // transect_width)

				#print(f"Num Transects: {num_transects}")
				#print(f"Cross Section width: {length}, Max Transect width: {transect_width}, Max Full Transects: {max_full_transects}")

				for i in range(max_full_transects+1):
					transect_coords[i].append(tuple(cross_section[1] - i*transect_width*direction))

				num_remaining_transects = num_transects + 1 - max_full_transects

				if num_remaining_transects > 1:
					last_coord = cross_section[1] - max_full_transects*transect_width*direction
					remaining_vec = last_coord - cross_section[0]
					remaining_dist = np.linalg.norm(remaining_vec)

					new_transect_width = remaining_dist / num_remaining_transects

					#print(f"Remaining Transects: {num_remaining_transects}, Remaining Dist: {remaining_dist}, New Transect Width: {new_transect_width}")

					for i in range(1,num_remaining_transects):
						#transect_coords[max_full_transects+i].append(tuple(last_coord - i*new_transect_width*direction))
						# Collapse all partial width transects down to bank 
						transect_coords[max_full_transects+i].append(tuple(cross_section[0]))

				
				transect_coords[-1].append(tuple(cross_section[0]))

			elif bias == 'pruned_inner_bank':
				# Bias towards inner bank and collapse redundant transects
				transect_width = self._sensor_radius * 2.
				max_full_transects = int(length // transect_width)

				print(f"Num Transects: {num_transects}")
				print(f"Cross Section width: {length}, Max Transect width: {transect_width}, Max Full Transects: {max_full_transects}")

				for i in range(max_full_transects+1):
					transect_coords[i].append(tuple(cross_section[1] - i*transect_width*direction))

				num_remaining_transects = num_transects + 1 - max_full_transects

				if num_remaining_transects > 1:
					last_coord = cross_section[1] - max_full_transects*transect_width*direction
					remaining_vec = last_coord - cross_section[0]
					remaining_dist = np.linalg.norm(remaining_vec)

					new_transect_width = remaining_dist / num_remaining_transects

					print(f"Remaining Transects: {num_remaining_transects}, Remaining Dist: {remaining_dist}, New Transect Width: {new_transect_width}")

					"""
					for i in range(1,num_remaining_transects):
						#transect_coords[max_full_transects+i].append(tuple(last_coord - i*new_transect_width*direction))
						# Collapse all partial width transects down to bank 
						#transect_coords[max_full_transects+i].append(tuple(cross_section[0]))
						# Trying to prune coincident transects
						if transect_coords[max_full_transects+i-1][-1] != tuple(cross_section[0]):
							transect_coords[max_full_transects+i].append(tuple(cross_section[0]))
					"""
					
					"""
					if num_remaining_transects > 1:
						transect_coords[max_full_transects+1].append(tuple(cross_section[0]))
					"""
				
				transect_coords[-1].append(tuple(cross_section[0]))

			# if bank_bias:
			# 	# Todo needs to stop before going past cross section length
			# 	transect_width = self._sensor_radius * 2.
				
			# 	for i in range(num_full_transects+1):
			# 		transect_coords[i].append(tuple(cross_section[1] - i*transect_width*direction))

			# 	num_remaining_transects = num_transects + 1 - num_full_transects

			# 	if num_remaining_transects > 1:
			# 		last_coord = cross_section[1] - num_full_transects*transect_width*direction
			# 		remaining_vec = last_coord - cross_section[0]
			# 		remaining_dist = np.linalg.norm(remaining_vec)

			# 		new_transect_width = remaining_dist / num_remaining_transects

			# 		print(f"Remaining Transects: {num_remaining_transects}, Remaining Dist: {remaining_dist}, New Transect Width: {new_transect_width}")
			# 		for i in range(1,num_remaining_transects):
			# 			transect_coords[num_full_transects+i].append(tuple(last_coord - i*new_transect_width*direction))
				
			# 	transect_coords[-1].append(tuple(cross_section[0]))

			else:
				transect_width = length / (num_transects+1)	

				for i in range(num_transects+2):
					transect_coords[i].append(tuple(cross_section[0] + i*transect_width*direction))


		constraints = [OpenConstraint(coord_list) for coord_list in transect_coords]

		return constraints