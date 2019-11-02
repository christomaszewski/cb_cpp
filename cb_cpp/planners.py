import numpy as np
import robot_primitives as rp

from . import layouts, refinements, sequencers, linkers

class LegacyConstraintBasedBoustrophedon(object):
	""" Deprecated, Use ConstraintBasedBoustrophedon instead """

	def __init__(self, vehicle_radius, sensor_radius=None, horizontal=False, **unknown_options):
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius if sensor_radius else vehicle_radius
		self._heuristic = rp.heuristics.EuclideanDistance()
		if horizontal:
			self._layout = layouts.HorizontalBoustrophedonPattern(vehicle_radius, sensor_radius)
		else:
			self._layout = layouts.BoustrophedonPattern(vehicle_radius, sensor_radius)
		self._refinements = [refinements.AlternatingDirections()]
		self._sequencer = sequencers.GreedySequencer(self._heuristic)
		self._linker = linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		print(f"num constraints: {len(constraints)}")
		for r in self._refinements:
			r.refine_constraints(constraints, area_ingress_point=area_ingress_point)
		print(constraints)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		print(constraint_chain)
		path = self._linker.link_constraints(constraint_chain)

		print(f"num waypoints: {len(path.coord_list)}")

		return path

class ConstraintBasedBoustrophedon(object):

	def __init__(self, vehicle_radius, sensor_radius, transect_orientation, **unknown_options):
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius if sensor_radius else vehicle_radius
		self._transect_orientation = transect_orientation
		self._heuristic = rp.heuristics.EuclideanDistance()
		self._layout = layouts.OrientedBoustrophedonPattern.from_transect_orientation(vehicle_radius, sensor_radius, transect_orientation)
		self._refinements = [refinements.AlternatingDirections()]
		self._sequencer = sequencers.GreedySequencer(self._heuristic)
		self._linker = linkers.SimpleLinker()

	@classmethod
	def horizontal(cls, vehicle_radius, sensor_radius=None, **options):
		return cls(vehicle_radius, sensor_radius, transect_orientation=(1,0), **options)

	@classmethod
	def vertical(cls, vehicle_radius, sensor_radius=None, **options):
		return cls(vehicle_radius, sensor_radius, transect_orientation=(0,1), **options)

	@classmethod
	def parallel_to_side(cls, vehicle_radius, sensor_radius, side):
		# Sweep orientation should be parallel to given side so compute direction of side
		side_direction = (side[1][0] - side[0][0], side[1][1] - side[0][1])

		return cls(vehicle_radius, sensor_radius, side_direction)

	@classmethod
	def perpendicular_to_side(cls, vehicle_radius, sensor_radius, side):
		# Sweep orientation is perpendicular to given side so compute side normal
		side_normal = (side[0][1] - side[1][1], side[1][0] - side[0][0])

		return cls(vehicle_radius, sensor_radius, side_normal)

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints, area_ingress_point=area_ingress_point)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path

class ConstraintBasedSpiral(object):

	def __init__(self, vehicle_radius, sensor_radius=None, **unknown_options):
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius if sensor_radius else vehicle_radius
		self._heuristic = rp.heuristics.EuclideanDistance()
		self._layout = layouts.SpiralPattern(vehicle_radius, sensor_radius)
		self._refinements = []
		self._sequencer = sequencers.GreedySequencer(self._heuristic)
		self._linker = linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints, area_ingress_point=area_ingress_point)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path


class DriftingBoustrophedon(object):

	def __init__(self, vehicle_radius, sensor_radius, flow_field, **unknown_options):
		""" Todo: Make sure this works is no flow_field is specified so we can supply default value to param above """
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius
		self._flow_field = flow_field
		self._heuristic = rp.heuristics.EuclideanDistance()
		self._layout = layouts.BoustrophedonPattern(vehicle_radius, sensor_radius)
		self._refinements = [refinements.AlternatingDirections()]
		self._refinements.append(refinements.DownstreamDrift(flow_field))
		self._sequencer = sequencers.GreedySequencer(self._heuristic)
		self._linker = linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints, area_ingress_point=area_ingress_point)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path


class EnergyEfficientBoustrophedon(object):

	def __init__(self, vehicle_radius, sensor_radius, flow_field, **unknown_options):
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius
		self._flow_field = flow_field
		self._sequencing_heuristic = rp.heuristics.EuclideanDistance()
		#rp.heuristics.OpposingFlowEnergy(flow_field)

		self._layout = layouts.BoustrophedonPattern(vehicle_radius, sensor_radius)
		self._refinements = [refinements.MaximizeFlowAlignment(flow_field)]
		self._sequencer = sequencers.MatchingSequencer(self._sequencing_heuristic)
		self._linker = linkers.SimpleLinker()

	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints, area_ingress_point=area_ingress_point)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path
		

class EnergyEfficientDrift(object):

	def __init__(self, vehicle_radius, sensor_radius, flow_field, **unknown_options):
		self._vehicle_radius = vehicle_radius
		self._sensor_radius = sensor_radius
		self._flow_field = flow_field
		self._sequencing_heuristic = rp.heuristics.OpposingFlowEnergy(flow_field)

		self._layout = layouts.BoustrophedonPattern(vehicle_radius, sensor_radius)
		self._refinements = [refinements.MaximizeFlowAlignment(flow_field)]
		self._refinements.append(refinements.DownstreamDrift(flow_field))
		self._sequencer = sequencers.GreedySequencer(self._sequencing_heuristic)
		self._linker = linkers.SimpleLinker()


	def plan_coverage_path(self, area, area_ingress_point=None):
		constraints = self._layout.layout_constraints(area)
		for r in self._refinements:
			r.refine_constraints(constraints, area_ingress_point=area_ingress_point)
		constraint_chain = self._sequencer.sequence_constraints(constraints, area_ingress_point)
		path = self._linker.link_constraints(constraint_chain)

		return path