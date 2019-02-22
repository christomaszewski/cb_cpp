import numpy as np
import robot_primitives as rp

import layout as c_layouts
import refinement as c_refinements
import sequence as c_sequencers
import link as c_linkers

class ConstraintBasedLawnmower:

	def __init__(self, sensor_radius, vehicle_radius, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius
		self._heuristic = rp.heuristics.EuclideanDistance()
		self._layout = c_layouts.LawnmowerPattern(sensor_radius, vehicle_radius)
		self._refinement = c_refinements.AlternatingDirections()
		self._sequencer = c_sequencers.GreedySequencer(self._heuristic)
		self._linker = c_linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		self._refinement.refine_constraints(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path


class DriftingLawnmower:

	def __init__(self, sensor_radius, vehicle_radius, flow_field, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius
		self._flow_field = flow_field
		self._heuristic = rp.heuristics.EuclideanDistance()
		self._layout = c_layouts.LawnmowerPattern(sensor_radius, vehicle_radius)
		self._first_refinement = c_refinements.AlternatingDirections()
		self._second_refinement = c_refinements.DownstreamDrift(flow_field)
		self._sequencer = c_sequencers.GreedySequencer(self._heuristic)
		self._linker = c_linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		self._first_refinement.refine_constraints(constraints)
		self._second_refinement.refine_constraints(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path


class EnergyEfficientCoverage:

	def __init__(self, sensor_radius, vehicle_radius, flow_field, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius
		self._flow_field = flow_field
		self._sequencing_heuristic = rp.heuristics.OpposingFlowEnergy(flow_field)

		self._layout = c_layouts.LawnmowerPattern(sensor_radius, vehicle_radius)
		self._refinement = c_refinements.OptimizedDrift(flow_field)
		self._sequencer = c_sequencers.GreedySequencer(self._sequencing_heuristic)
		self._linker = c_linkers.SimpleLinker()


	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		self._refinement.refine_constraints(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path