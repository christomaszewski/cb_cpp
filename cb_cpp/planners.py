import numpy as np
import robot_primitives as rp

from . import layouts, refinements, sequencers, linkers

class ConstraintBasedBoustrophedon(object):

	def __init__(self, sensor_radius, vehicle_radius, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius
		self._heuristic = rp.heuristics.EuclideanDistance()
		self._layout = layouts.BoustrophedonPattern(sensor_radius, vehicle_radius)
		self._refinements = [refinements.AlternatingDirections()]
		self._sequencer = sequencers.GreedySequencer(self._heuristic)
		self._linker = linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path


class ConstraintBasedSpiral(object):

	def __init__(self, sensor_radius, vehicle_radius, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius
		self._heuristic = rp.heuristics.EuclideanDistance()
		self._layout = layouts.SpiralPattern(sensor_radius, vehicle_radius)
		self._refinements = []
		self._sequencer = sequencers.GreedySequencer(self._heuristic)
		self._linker = linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path


class DriftingBoustrophedon(object):

	def __init__(self, sensor_radius, vehicle_radius, flow_field, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius
		self._flow_field = flow_field
		self._heuristic = rp.heuristics.EuclideanDistance()
		self._layout = layouts.BoustrophedonPattern(sensor_radius, vehicle_radius)
		self._refinements = [refinements.AlternatingDirections()]
		self._refinements.append(refinements.DownstreamDrift(flow_field))
		self._sequencer = sequencers.GreedySequencer(self._heuristic)
		self._linker = linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path


class EnergyEfficientBoustrophedon(object):

	def __init__(self, sensor_radius, vehicle_radius, flow_field, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius
		self._flow_field = flow_field
		self._sequencing_heuristic = rp.heuristics.OpposingFlowEnergy(flow_field)

		self._layout = layouts.BoustrophedonPattern(sensor_radius, vehicle_radius)
		self._refinements = [refinements.MaximizeFlowAlignment(flow_field)]
		self._sequencer = sequencers.GreedySequencer(self._sequencing_heuristic)
		self._linker = linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path
		

class EnergyEfficientDrift(object):

	def __init__(self, sensor_radius, vehicle_radius, flow_field, **unknown_options):
		self._sensor_radius = sensor_radius
		self._vehicle_radius = vehicle_radius
		self._flow_field = flow_field
		self._sequencing_heuristic = rp.heuristics.OpposingFlowEnergy(flow_field)

		self._layout = layouts.BoustrophedonPattern(sensor_radius, vehicle_radius)
		self._refinements = [refinements.MaximizeFlowAlignment(flow_field)]
		self._refinements.append(refinements.DownstreamDrift(flow_field))
		self._sequencer = sequencers.GreedySequencer(self._sequencing_heuristic)
		self._linker = linkers.SimpleLinker()


	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path