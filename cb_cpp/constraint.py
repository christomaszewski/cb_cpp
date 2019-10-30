import shapely.geometry
import numpy as np

from .base import Constraint

class BasicConstraint(Constraint):
	""" An abstract constraint class that defines common methods used by OpenConstraint
		 and ClosedConstraint implementations. Should not be instantiated.
	"""

	def __init__(self, coord_list, **constrained_parameters):
		self._coord_list = coord_list.copy()
		self._constrained_parameters = {**constrained_parameters}

		for param in self._constrained_parameters.keys():
			setattr(BasicConstraint, param, self._property_factory(param))

	def _property_factory(self, parameter):
		return property(lambda obj:obj._constrained_parameters[parameter], 
							lambda obj, val: obj._constrained_parameters.update({parameter:value}), 
							lambda obj:obj._constrained_parameters.pop(parameter))

	def is_constrained(self, parameter):
		return parameter in self._constrained_parameters

	def constrain_parameter(self, parameter, value):
		self._constrained_parameters[parameter] = value
		setattr(BasicConstraint, parameter, self._property_factory(parameter))

	def unconstrain_parameter(self, parameter):
		if parameter in self._constrained_parameters:
			del self._constrained_parameters[parameter]
			delattr(BasicConstraint, parameter)
			return True
		else:
			print(f"Error: Parameter {parameter} not constrained")
			return False

	@property
	def constrained_parameters(self):
		return self._constrained_parameters

	@property
	def coord_list(self):
		return self._coord_list
	

class OpenConstraint(BasicConstraint):

	def __init__(self, coord_list, **constrained_parameters):
		super().__init__(coord_list, **constrained_parameters)

		# Extra constraint endpoints, handling singleton constraint special case
		if len(coord_list) == 1:
			first = last = coord_list[0]
		else:
			first, *_, last = coord_list
		
		self._endpoints = (first, last)

	def __repr__(self):
		return str(self._coord_list)

	def get_coord_list(self, ingress_point=None, **unknown_parameters):
		# If direction is constrained
		if 'direction' in self._constrained_parameters:
			direction = self._constrained_parameters['direction']
			# If ingress_point is specified and differs from direction constraint
			if ingress_point is not None and ingress_point != self._endpoints[direction[0]]:
				print('Error: specified ingress_point violates direction constraint')
				return None
			elif direction[0] == 0:
				return self._coord_list
			else:
				return list(reversed(self._coord_list))
		
		# Direction is not constrained but ingress_point specified
		elif ingress_point is not None:
			try:
				endpoint_index = self._endpoints.index(ingress_point)
				if endpoint_index == 0:
					return self._coord_list
				else:
					return reversed(self._coord_list)
			except ValueError:
				print(f"Error: specified ingress_point {ingress_point} not found in constraint ingress_points")
				return None
			except:
				print('Error: an unknown error occurred')
				return None

		# Direction not constrained and no ingress_point specified, just return coords
		else:
			return self._coord_list

	def select_ingress(self, ingress_point):
		""" For open constraints choosing an ingress point implicitly constrains
			 the direction of the constraint. If specified ingress_point is not a
			 valid ingress point for this constraint, return false.
		"""
		try:
			ingress_index = self.ingress_points.index(ingress_point)

			if not self.is_constrained('direction'):
				direction = [ingress_index, (ingress_index+1)%2]

				self.constrain_parameter('direction', direction)

			return True
		except ValueError:
			print(f"Error: specified ingress_point {ingress_point} not found in constraint ingress_points")
			return False
		except:
			print('Error: an unknown error occurred while trying to select ingress')
			return False

	@property
	def size(self):
		return len(self._coord_list)
	
	@property
	def endpoints(self):
		return self._endpoints
	
	@property
	def ingress_points(self):
		if 'direction' in self._constrained_parameters:
			direction = self._constrained_parameters['direction']
			return [self._endpoints[direction[0]]]
		else:
			return self._endpoints

	@property
	def egress_points(self):
		if 'direction' in self._constrained_parameters:
			direction = self._constrained_parameters['direction']
			return [self._endpoints[direction[1]]]
		else:
			return self._endpoints
	

class ClosedConstraint(BasicConstraint):
	""" Class to represent a closed loop constraint """

	def get_coord_list(self, ingress_point=None, endpoint_offset=0.0, **unknown_parameters):
		if ingress_point:
			# currently saves ingress_point choice, maybe shouldn't do this
			if not self.select_ingress(ingress_point):
				return None
		elif len(self.ingress_points) > 1:
			# take first ingress point for now
			ingress_point = self.ingress_points[0]
		else:
			ingress_point = self.ingress_points[0]

		transition_index = self._coord_list.index(ingress_point)
		step = 1
		if self.is_constrained('direction') and self._constrained_parameters['direction'][0] != 0:
			step = -1

		final_segment = np.asarray(self._coord_list[transition_index]) - np.asarray(self._coord_list[(transition_index-step)%len(self._coord_list)])
		final_segment /= np.linalg.norm(final_segment)

		endpoint = self._coord_list[transition_index] - endpoint_offset * final_segment

		coords = [*self._coord_list[transition_index::step], *self._coord_list[:transition_index:step], tuple(endpoint)]
		return coords

	def select_ingress(self, ingress_point):
		""" For closed constraints choosing the ingress point does not constrain direction """
		try:
			ingress_index = self.ingress_points.index(ingress_point)

			if not self.is_constrained('transition'):
				self.constrain_parameter('transition', [self._coord_list[ingress_index]])
			else:
				self._constrained_parameters['transition'] = [self._constrained_parameters['transition'][ingress_index]]

			return True
		except ValueError:
			print(f"Error: specified ingress_point {ingress_point} not found in constraint ingress_points")
			return False
		except:
			print('Error: an unknown error occurred')
			return False

	@property
	def ingress_points(self):
		if "transition" in self._constrained_parameters:
			return self._constrained_parameters["transition"]
		else:
			return self._coord_list

	@property
	def egress_points(self):
		if "transition" in self._constrained_parameters:
			return self._constrained_parameters["transition"]
		else:
			return self._coord_list
