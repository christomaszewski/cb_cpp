import collections
import robot_primitives as rp

from .base import ConstraintLinker

class SimpleLinker(ConstraintLinker):
	""" Simply connects each constraint egress to the following constraint's ingress point """

	def link_constraints(self, constraint_chain, ingress_point=None, offset=0.0):
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