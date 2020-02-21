from collections import defaultdict
import itertools
import numpy as np

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
			print('No start point specified, choosing first constraint in list')
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


class MatchingSequencer(ConstraintSequencer):

	def __init__(self, heuristic):
		# The heuristic is used to chose the next constraint and its ingress point
		self._heuristic = heuristic

	def sequence_constraints(self, constraints, start_point=None):
		# Partition constraints by direction
		constraint_partitions = defaultdict(set)

		for c in constraints:
			if not c.is_constrained('direction'):
				print('Must provide directed constraints to MatchingSequencer')
				return []
			else:
				constraint_partitions[tuple(c.direction)].add(c)

		if len(constraint_partitions) != 2:
			print('Must provide constraints with exactly two different directions to MatchingSequencer')
			return []

		partition_sizes = {k:len(v) for k, v in constraint_partitions.items()}
		num_constraints = list(partition_sizes.values())

		starting_constraint = None
		ingress_point = None
		ingress_point_index = None
		if start_point is None:
			# No start_point specified, chose the partition with more constraints if possible
			partition_key = max(partition_sizes, key=partition_sizes.get)
			starting_constraint = constraint_partitions[partition_key][0]
			ingress_point = starting_constraint.ingress_points[0]
			ingress_point_index = 0
		else:
			# Do a search to find constraint with ingress points closest to start_point 
			min_cost = None

			# If one partition has more constraints then another limit search to that partition
			search_partitions = []
			if num_constraints[0] != num_constraints[1]:
				search_partitions.append(constraint_partitions[max(partition_sizes, key=partition_sizes.get)])
			else:
				search_partitions = constraint_partitions.values()

			for partition in search_partitions:
				for c in partition:
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

		constraint_partitions[tuple(starting_constraint.direction)].remove(starting_constraint)

		remaining_constraints = [len(p) for p in constraint_partitions.values()]
		while any(remaining_constraints):
			# Find next closest ingress point on available constraints
			next_constraint = None
			ingress_point = None
			ingress_point_index = None
			min_cost = None
			secondary_cost = None
			next_direction = tuple(constraint_chain[-1].direction[::-1])
			for c in constraint_partitions[next_direction]:
				for idx, pt in enumerate(c.ingress_points):
					cost = self._heuristic.compute_cost(chain_egress_pt, pt)
					if min_cost is not None and cost == min_cost:
							tiebreaker_current = self._heuristic.compute_cost(ingress_point, constraint_chain[0].ingress_points[0])
							tiebreaker_new = self._heuristic.compute_cost(pt, constraint_chain[0].ingress_points[0])
							print(f"equal cost constaints, choosing constraint furthest from area ingress point {tiebreaker_current} {tiebreaker_new}")
							if tiebreaker_new > tiebreaker_current:
								print('switching to new constraint')
								min_cost = cost
								next_constraint = c
								ingress_point = pt
								ingress_point_index = idx
										
					elif min_cost is None or cost < min_cost:
						min_cost = cost
						next_constraint = c
						ingress_point = pt
						ingress_point_index = idx

			constraint_partitions[next_direction].remove(next_constraint)

			next_constraint.select_ingress(ingress_point)

			constraint_chain.append(next_constraint)
			(chain_egress_pt,) = next_constraint.egress_points

			remaining_constraints = [len(p) for p in constraint_partitions.values()]


		return constraint_chain 


class BruteForceMatchingSequencer(ConstraintSequencer):

	def __init__(self):
		pass

	def sequence_constraints(self, constraints):
		# Partition constraints by direction
		constraint_partitions = defaultdict(set)

		for c in constraints:
			if not c.is_constrained('direction'):
				print('Must provide directed constraints to BruteForceMatchingSequencer')
				return []
			else:
				constraint_partitions[tuple(c.direction)].add(c)

		if len(constraint_partitions) != 2:
			print('Must provide constraints with exactly two different directions to BruteForceMatchingSequencer')
			return []

		partition_sizes = {k:len(v) for k, v in constraint_partitions.items()}
		num_constraints = list(partition_sizes.values())

		print(f"Constraint partitions contain {num_constraints[0]} and {num_constraints[1]} constraints")

		constraint_permutations = []
		for partition in constraint_partitions.values():
			constraint_permutations.append(list(itertools.permutations(iter(partition))))

		print(f"Found {len(constraint_permutations[0])} permutations of partition 1")
		print(f"Found {len(constraint_permutations[1])} permutations of partition 2")

		print('Generating all possible constraint chains...')
		constraint_chains = []
		for p1 in constraint_permutations[0]:
			for p2 in constraint_permutations[1]:
				#print(f"Length of each permutation:{len(p1)}, {len(p2)}")
				
				#chain = list(next(it) for it in itertools.cycle(iters))
				if len(p1) >= len(p2):
					iters = [iter(p1), iter(p2)]
					cycle = itertools.cycle(iters)
					#print('p1 p2 cycle success')
					chain = list(next(it) for it in itertools.cycle(iters))
					yield chain
					#constraint_chains.append(cycle)
				
				#chain = list(next(it) for it in itertools.cycle(iters))
				if len(p2) >= len(p1):
					iters = [iter(p2), iter(p1)]
					cycle = itertools.cycle(iters)
					#print('p2 p1 cycle success')
					chain = list(next(it) for it in itertools.cycle(iters))
					yield chain
					#constraint_chains.append(cycle)
					
		#return constraint_chains