from abc import ABC, abstractmethod
from enum import Enum

class Constraint(ABC):

	@abstractmethod
	def select_ingress(self, ingress_point):
		""" Method to adjust constraints appropriately once desired ingress point is known """
		raise NotImplementedError()

	@abstractmethod
	def get_coord_list(self, ingress_point):
		""" All Constraints must have coordinate list representation available """
		raise NotImplementedError()

	@property
	@abstractmethod
	def constrained_parameters(self):
		""" All Constraints must provide set of constrained parameters to check compatibility with some algorithms """
		raise NotImplementedError()

	@property
	@abstractmethod
	def ingress_points(self):
		""" All Constraints must provide valid ingress points """
		raise NotImplementedError()

	@property
	@abstractmethod
	def egress_points(self):
		""" All Constraints must provide valid egress points """
		raise NotImplementedError()

	@classmethod
	def __subclasshook__(cls, C):
		if cls is Constraint:
			attrs = set(dir(C))
			if set(cls.__abstractmethods__) <= attrs:
				return True

		return NotImplemented


class ConstraintLayout(ABC):

	@abstractmethod
	def layout_constraints(self, area):
		""" All ConstraintArchitects must provide a method that returns a list of constraints given an area """
		raise NotImplementedError()

	@classmethod
	def __subclasshook__(cls, C):
		if cls is ConstraintLayout:
			attrs = set(dir(C))
			if set(cls.__abstractmethods__) <= attrs:
				return True

		return NotImplemented


class ConstraintRefinement(ABC):

	@abstractmethod
	def refine_constraints(self, constraints):
		""" Refinements adjust constraints in diffrent ways depending on other inputs """
		raise NotImplementedError()

	@classmethod
	def __subclasshook__(cls, C):
		if cls is ConstraintRefinement:
			attrs = set(dir(C))
			if set(cls.__abstractmethods__) <= attrs:
				return True

		return NotImplemented


class ConstraintSequencer(ABC):

	@abstractmethod
	def sequence_constraints(self, constraints):
		""" Build ordered chain of constraints with assigned ingress/egress points """
		raise NotImplementedError()

	@classmethod
	def __subclasshook__(cls, C):
		if cls is ConstraintSequencer:
			attrs = set(dir(C))
			if set(cls.__abstractmethods__) <= attrs:
				return True

		return NotImplemented


class ConstraintLinker(ABC):

	@abstractmethod
	def link_constraints(self, constraints):
		""" Process constraints and link them into a single path using specified algorithm """
		raise NotImplementedError()

	@classmethod
	def __subclasshook__(cls, C):
		if cls is ConstraintLinker:
			attrs = set(dir(C))
			if set(cls.__abstractmethods__) <= attrs:
				return True

		return NotImplemented