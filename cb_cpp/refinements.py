import numpy as np
import itertools
from sortedcontainers import SortedList

import robot_primitives as rp

from .base import ConstraintRefinement

class AlternatingDirections(ConstraintRefinement):

	def refine_constraints(self, constraints, area_ingress_point=None, starting_direction=[0,1], **unknown_options):
		starting_constraint_idx = 0

		# Find/select closest constraint to ingress point
		if area_ingress_point is not None:
			starting_constraint = None
			min_cost = None

			for c_idx, c in enumerate(constraints):
				for pt_idx, pt in enumerate(c.ingress_points):
					cost = rp.heuristics.EuclideanDistance.compute_cost(area_ingress_point, pt)
					if min_cost is None or cost < min_cost:
						min_cost = cost
						starting_constraint = c
						starting_constraint_idx = c_idx
						ingress_point = pt
						ingress_point_idx = pt_idx

			starting_constraint.select_ingress(ingress_point)
			starting_direction = starting_constraint.direction.copy()

		current_direction = starting_direction.copy()

		for c in constraints[starting_constraint_idx:]:
			c.constrain_parameter('direction', current_direction.copy())
			current_direction.reverse()

		current_direction = starting_direction.copy()

		for c in constraints[starting_constraint_idx::-1]:
			c.constrain_parameter('direction', current_direction.copy())
			current_direction.reverse()
		
		return constraints


class DownstreamDrift(ConstraintRefinement):

	def __init__(self, flow_field):
		self._flow_field = flow_field

	def refine_constraints(self, constraints, default_thrust=(0.,1.), **unknown_options):
		for c in constraints:
			if not c.is_constrained('direction'):
				print('Error: Constraint direction is unconstrained')
				return None

			direction = c.direction
			endpoints = c.endpoints

			constraint_direction = np.asarray(endpoints[direction[1]]) - np.asarray(endpoints[direction[0]])
			constraint_direction /= np.linalg.norm(constraint_direction)

			# use flow direction at ingress point of constraint for now
			flow_direction = np.asarray(self._flow_field[endpoints[direction[0]]])
			flow_direction /= np.linalg.norm(flow_direction)

			# Set thrust constraint to full allowable range of thrust fractions 
			# for first coordinate because any thrust should be allowed to arrive at 
			# ingress point of constraint
			c.constrain_parameter('thrust', [default_thrust])

			# If constraint direction corresponds to flow direction
			if np.dot(constraint_direction, flow_direction) > 0:
				# add (0,0) thrust constraint so no thrust is applied to all subsequent coords
				c.thrust.extend(itertools.repeat((0.,0.), c.size-1))
			else:
				c.thrust.extend(itertools.repeat(default_thrust, c.size-1))

			print(f"thrust: {c.thrust} direction: {direction}, endpoints: {endpoints}")

		return constraints


class MaximizeFlowAlignment(ConstraintRefinement):

	def __init__(self, flow_field, nominal_speed=0.5, delta=0.01):
		self._flow_field = flow_field
		self._nominal_speed = nominal_speed
		self._energy_heuristic = rp.heuristics.OpposingFlowEnergy(flow_field, delta)

	def refine_constraints(self, constraints, default_thrust=(0.,1.), **unknown_options):
		constraint_costs = []
		constraint_idx = []

		for idx, c in enumerate(constraints):
			# assumes all constraints have coords orders similarly
			# probably want to check both directions or use heuristic that is agnostic to direction
			coords = c.coord_list
			print(f"Computing cost of Constraint {idx} of {len(constraints)}")
			cost = 0.
			for start, end in zip(coords, coords[1:]):
				cost += self._energy_heuristic.compute_cost(start, end, self._nominal_speed)

			constraint_costs.append(cost)
			constraint_idx.append(idx)

			print(f"Constraint {idx} has cost of {cost}")

		#print(constraint_costs)

		sorted_constraints = SortedList(constraint_idx, key=lambda i:constraint_costs[i])

		split_index = np.ceil(len(sorted_constraints) / 2.).astype(int)
		print(split_index, len(sorted_constraints[split_index:]), len(sorted_constraints[:split_index]))
		# constrain the direction of constraints to lie with the flow where fastest
		# and against the flow where the flow is slowest
		for index in sorted_constraints[split_index:]:
			c = constraints[index]
			if not c.is_constrained('direction'):
				c.constrain_parameter('direction', [1,0])
			else: 
				c.direction = [1,0]

		for index in sorted_constraints[:split_index]:
			c = constraints[index]
			
			if not c.is_constrained('direction'):
				c.constrain_parameter('direction', [0,1])
			else: 
				c.direction = [0,1]

		return constraints


class OptimizedDrift(ConstraintRefinement):

	def __init__(self, flow_field):
		self._flow_field = flow_field
		self._energy_heuristic = rp.heuristics.OpposingFlowEnergy(flow_field)

	def refine_constraints(self, constraints, default_thrust=(0.,1.), **unknown_options):
		constraint_costs = []
		constraint_idx = []

		for idx, c in enumerate(constraints):
			# assumes all constraints have coords orders similarly
			coords = c.coord_list
			#print('computing cost of constraint', idx)
			cost = 0.
			for start, end in zip(coords, coords[1:]):
				cost += self._energy_heuristic.compute_cost(start, end)

			constraint_costs.append(cost)
			constraint_idx.append(idx)

		print(constraint_costs)

		sorted_constraints = SortedList(constraint_idx, key=lambda i:constraint_costs[i])

		split_index = np.ceil(len(sorted_constraints) / 2).astype(int)
		
		# constrain the direction and thrust of drift constraints
		for index in sorted_constraints[split_index:]:
			c = constraints[index]
			if not c.is_constrained('direction'):
				c.constrain_parameter('direction', [1,0])
			else: 
				c.direction = [1,0]

			c.constrain_parameter('thrust', [default_thrust])
			c.thrust.extend(itertools.repeat((0.,0.), c.size-1))

		for index in sorted_constraints[:split_index]:
			c = constraints[index]
			
			if not c.is_constrained('direction'):
				c.constrain_parameter('direction', [0,1])
			else: 
				c.direction = [0,1]

			c.constrain_parameter('thrust', [default_thrust])
			c.thrust.extend(itertools.repeat(default_thrust, c.size-1))

		return constraints