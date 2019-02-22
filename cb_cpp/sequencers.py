from .base import ConstraintSequencer

class GreedySequencer(ConstraintSequencer):

	def __init__(self, heuristic):
		# The heuristic is used to chose the next constraint and its ingress point
		self._heuristic = heuristic

	def sequence_constraints(self, constraints, start_point=None):
		starting_constraint = None
		ingress_point = None
		ingress_point_index = None
		if start_point is None:
			# No start_point specified so take first ingress point of first constraint
			starting_constraint = constraints[0]
			ingress_point = starting_constraint.ingress_points[0]
			ingress_point_index = 0
		else:
			# Do a search to find constraint with ingress points closest to start_point 
			min_cost = None
			for c in constraints:
				for idx, pt in enumerate(c.ingress_points):
					cost = self._heuristic.compute_cost(start_point, pt)
					if min_cost is None or cost < min_cost:
						min_cost = cost
						starting_constraint = c
						ingress_point = pt
						ingress_point_index = idx

		starting_constraint.select_ingress(ingress_point)

		# Setup vars to store chain of constraints
		constraint_chain = [starting_constraint]
		# Extract egress point from starting_constraint. Will fail if egress_points has more than 1 element
		(chain_egress_pt,) = starting_constraint.egress_points
		#chain_egress_pt = starting_constraint.get_coord_list()[-1]

		unchained_constraints = set(constraints)
		unchained_constraints.remove(starting_constraint)

		while len(unchained_constraints) > 0:
			# Find next closest ingress point on available constraints
			next_constraint = None
			ingress_point = None
			ingress_point_index = None
			min_cost = None
			for c in unchained_constraints:
				for idx, pt in enumerate(c.ingress_points):
					cost = self._heuristic.compute_cost(chain_egress_pt, pt)
					if min_cost is None or cost < min_cost:
						min_cost = cost
						next_constraint = c
						ingress_point = pt
						ingress_point_index = idx

			unchained_constraints.remove(next_constraint)

			next_constraint.select_ingress(ingress_point)

			constraint_chain.append(next_constraint)
			(chain_egress_pt,) = next_constraint.egress_points


		return constraint_chain 