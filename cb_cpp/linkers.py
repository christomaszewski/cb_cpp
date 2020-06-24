import collections

import robot_primitives as rp
import robot_utils as rut

from .base import ConstraintLinker

class SimpleLinker(ConstraintLinker):
	""" Simply connects each constraint egress to the following constraint's ingress point """

	def link_constraints(self, constraint_chain, domain=None, ingress_point=None, offset=0.0, **unknown_options):
		coords = []
		if ingress_point is not None:
			coords.append(tuple(ingress_point))

		path_constraints = collections.defaultdict(list)

		for c in constraint_chain:
			new_coords = c.get_coord_list(endpoint_offset=offset)
			if new_coords is not None:
				coords.extend(new_coords)
			else:
				print('Error: Could not determine direction on constraint in chain')

			# assumes each constraint has the same parameters constrained for now
			for param, param_value in c.constrained_parameters.items():
				if param == 'direction':
					# Skip direction constraints since they aren't necessary in a final path
					continue
				else:
					path_constraints[param].extend(param_value)
		final_path = rp.paths.ConstrainedPath(coords, **path_constraints)

		return final_path

class AStarLinker(ConstraintLinker):
	""" Plans a clear path from each constraint's egress to the next constraints ingress point
		 using a A* Post-Smoothed Planner
	"""

	def link_constraints(self, constraint_chain, domain, ingress_point=None, egress_point=None, **unknown_options):
		path_planner = rut.planning.AStarPS(domain, rp.heuristics.EuclideanDistance, 5.0, 0.5)

		coords = []
		path_constraints = collections.defaultdict(list)

		for c in constraint_chain:
			new_coords = c.get_coord_list()

			# If we have a previous coordinate, plan path to first new coordinate
			if len(coords) > 0 and len(new_coords) > 0:
				linking_path = path_planner.plan_path(coords[-1], new_coords[0])
				
				if len(linking_path) > 2:
					connecting_coords = linking_path[1:-1]

					coords.extend(connecting_coords)
					# assumes each constraint has the same parameters constrained for now
					for param, param_value in c.constrained_parameters.items():
						if param == 'direction':
							# Skip direction constraints since they aren't necessary in a final path
							continue
						else:
							path_constraints[param].extend([None]*len(connecting_coords))

			coords.extend(new_coords)

			# assumes each constraint has the same parameters constrained for now
			for param, param_value in c.constrained_parameters.items():
				if param == 'direction':
					# Skip direction constraints since they aren't necessary in a final path
					continue
				else:
					path_constraints[param].extend(param_value)

		final_path = rp.paths.ConstrainedPath(coords, **path_constraints)

		return final_path
