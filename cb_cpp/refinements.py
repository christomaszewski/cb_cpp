import numpy as np
import itertools
from sortedcontainers import SortedList

import robot_primitives as rp

from .base import ConstraintRefinement

class AlternatingDirections(ConstraintRefinement):

	def refine_constraints(self, constraints, starting_direction=[0,1]):
		current_direction = starting_direction

		for c in constraints:
			c.constrain_parameter('direction', current_direction.copy())
			current_direction.reverse()

		return constraints


class DownstreamDrift(ConstraintRefinement):

	def __init__(self, flow_field):
		self._flow_field = flow_field

	def refine_constraints(self, constraints, default_thrust=(0.,1.)):
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


class OptimizedDrift(ConstraintRefinement):

	def __init__(self, flow_field):
		self._flow_field = flow_field
		self._energy_heuristic = rp.heuristics.OpposingFlowEnergy(flow_field)

	def refine_constraints(self, constraints, default_thrust=(0.,1.)):
		constraint_costs = []
		constraint_idx = []

		for idx, c in enumerate(constraints):
			# assumes all constraints have coords orders similarly
			coords = c.coord_list
			print('computing cost of constraint', idx)
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