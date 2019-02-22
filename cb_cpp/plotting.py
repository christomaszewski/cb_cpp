import matplotlib.pyplot as plt
import matplotlib
import numpy as np
from descartes.patch import PolygonPatch
import itertools

class DomainView(object):

	def __init__(self, domain, pause=0.0001):
		self._domain = domain
		x_min, y_min, x_max, y_max = domain.bounds
		x_dist = 1.0
		y_dist = (y_max - y_min)/x_max - x_min

		multiplier = 10

		plt.ion()
		self._fig = plt.figure(figsize=(x_dist*multiplier,y_dist*multiplier), dpi=100) # todo make this use domain size
		self._ax = None

		self._pause_length = pause
		self._title = 'Untitled'
		self._annotation = None

	def _draw(self):
		plt.axis('off')
		plt.show()
		plt.pause(self._pause_length)

	def reset_figure_size(self, size):
		self._fig.set_size_inches(*size)

		self._draw()	

	def clear_figure(self):
		self._fig.clf()
		self._ax = self._fig.add_subplot(1,1,1)
		self._ax.axis('equal')

	def plot_polygon(self, polygon, face_color, edge_color=None, alpha=1.0, zorder=0):
		if edge_color is None:
			edge_color = face_color
		patch = PolygonPatch(polygon, facecolor=face_color, edgecolor=edge_color, alpha=alpha, zorder=zorder)
		self._ax.add_patch(patch)

		x_min, y_min, x_max, y_max = polygon.bounds

		self._ax.set_xlim(x_min-1, x_max+1)
		self._ax.set_ylim(y_min-1, y_max+1)

		self._draw()

	def plot_domain(self, new_domain=None, domain_bg='xkcd:water blue', obstacle_bg='xkcd:reddish'):
		if new_domain is not None:
			self._domain = new_domain

		if self._domain is None:
			# No valid domain specified
			return

		self.clear_figure()

		domain_patch = PolygonPatch(self._domain.polygon, facecolor=domain_bg, edgecolor=domain_bg, alpha=1.0, zorder=-10)
		self._ax.add_patch(domain_patch)

		for o in self._domain.obstacles.values():
			obstacle_patch = PolygonPatch(o.polygon, facecolor=obstacle_bg, edgecolor=obstacle_bg, alpha=1.0, zorder=-5)
			self._ax.add_patch(obstacle_patch)

		x_min, y_min, x_max, y_max = self._domain.bounds

		self._ax.set_xlim(x_min-1, x_max+1)
		self._ax.set_ylim(y_min-1, y_max+1)

		self._draw()

	def plot_ingress_egress(self, ingress, egress):
		self._ax.plot(ingress[0], ingress[1], 'P', color='xkcd:vivid green', markersize=20)
		self._ax.plot(egress[0], egress[1], 'X', color='xkcd:tomato', markersize=20)

	def plot_configuration_space(self, vehicle_radius, domain_bg='xkcd:water blue', obstacle_bg='xkcd:reddish'):
		self.clear_figure()

		domain_patch = PolygonPatch(self._domain.polygon, facecolor=domain_bg, edgecolor=domain_bg, alpha=1.0, zorder=-10)
		self._ax.add_patch(domain_patch)

		offset_boundary, offset_obstacles = self._domain.get_configuration_space(vehicle_radius)

		domain_patch = PolygonPatch(offset_boundary, facecolor='xkcd:dark mint green', edgecolor='xkcd:dark mint green', alpha=1.0, zorder=-10)
		self._ax.add_patch(domain_patch)

		for o in offset_obstacles:
			obstacle_patch = PolygonPatch(o, facecolor=obstacle_bg, edgecolor=obstacle_bg, alpha=1.0, zorder=-5)
			self._ax.add_patch(obstacle_patch)

		for o in self._domain.obstacles.values():
			obstacle_patch = PolygonPatch(o.polygon, facecolor=obstacle_bg, edgecolor='xkcd:black', alpha=1.0, zorder=-5)
			self._ax.add_patch(obstacle_patch)


		x_min, y_min, x_max, y_max = offset_boundary.bounds

		self._ax.set_xlim(x_min-1, x_max+1)
		self._ax.set_ylim(y_min-1, y_max+1)

		self._draw()

	def center_view_to_domain(self):
		x_min, y_min, x_max, y_max = self._domain.bounds

		self._ax.set_xlim(x_min-1, x_max+1)
		self._ax.set_ylim(y_min-1, y_max+1)

		self._draw()

	def plot_sweep_slice(self, sweep_slice):
		colors = ['Green', 'Red', 'Black'] 

		for segment in sweep_slice:
			x, y = segment.xy
			color = colors[segment.type.value]
			self._ax.plot(x, y, 'o', color=color, zorder=1)
			self._ax.plot(x, y, color=color, alpha=0.7, linewidth=3, solid_capstyle='round', zorder=1)

		self._draw()

	def plot_constraint(self, constraint, annotation=None):
		colors = ['xkcd:silver', 'xkcd:spruce', 'xkcd:tomato'] 
		coords = constraint.get_coord_list()
		x,y = zip(*coords)
		direction = None
		if constraint.is_constrained('direction'):
			direction = constraint.direction
		color_id = 0

		if direction is not None:
			color_id = direction[0] + 1

		color = colors[color_id]
		#self._ax.plot(x, y, 'o', color=color, zorder=1)
		self._ax.plot(x, y, color=color, linewidth=5, solid_capstyle='round', zorder=1)

		
		endpoints = constraint.get_coord_list()[:2]
		midpoint = (np.asarray(endpoints[0]) + np.asarray(endpoints[1]))/2.

		if direction is None:
			#self._ax.arrow(midpoint[0], midpoint[1], 0, 0.5, color=color, shape='full', lw=0, length_includes_head=True, head_width=.3)
			#self._ax.arrow(midpoint[0], midpoint[1], 0, -0.5, color=color, shape='full', lw=0, length_includes_head=True, head_width=.3)
			#self._ax.arrow(endpoints[1][0], endpoints[1][1]-0.4, 0, 0.5, color=color, shape='full', lw=0, length_includes_head=True, head_width=.5)
			#self._ax.arrow(endpoints[0][0], endpoints[0][1]+0.4, 0, -0.5, color=color, shape='full', lw=0, length_includes_head=True, head_width=.5)
			pass
		else:
			offset = 0.4 if direction[1] == 0 else -0.4
			self._ax.arrow(endpoints[direction[1]][0], endpoints[direction[1]][1] + offset, 0, 0.5-direction[0], color=color, shape='full', lw=0, length_includes_head=True, head_width=.5)


		if annotation is not None:
			offset_midpoint = (midpoint[0]+0.05, midpoint[1])
			self._ax.annotate(annotation, offset_midpoint, fontsize=30, fontweight='bold')
		
		self._draw()

	def plot_path(self, path):
		undefined_color = 'xkcd:silver'
		color_map = matplotlib.cm.get_cmap('Spectral')

		coord_pairs = zip(path.coord_list, path.coord_list[1:])
		segment_colors = []
		if path.is_constrained('thrust'):
			segment_colors.extend(map(lambda thrust_range: undefined_color if thrust_range is None else color_map(np.mean(thrust_range)), path.thrust[1:]))
		else:
			segment_colors.extend(itertools.repeat(undefined_color, path.size-1))

		print(path.coord_list)
		x,y = zip(*path.coord_list)
		self._ax.plot(x, y, 'o', color=undefined_color, markersize=4, zorder=1)
		self._ax.plot(x[0], y[0], marker=5, color='xkcd:kiwi green', markersize=25)
		self._ax.plot(x[-1], y[-1], marker=9, color='xkcd:tomato', markersize=25)

		for seg_coords, seg_color in zip(coord_pairs, segment_colors):
			x,y = zip(*seg_coords)
			self._ax.plot(x, y, color=seg_color, linewidth=5, solid_capstyle='round', zorder=1)
		
		self._draw()


	def plot_cell(self, cell, color='Green', alpha=1.0, zorder=0, with_label=True, adjust_limits=False):
		cell_patch = PolygonPatch(cell.polygon, facecolor=color, edgecolor=color, alpha=alpha, zorder=zorder)
		self._ax.add_patch(cell_patch)

		if with_label:
			plt.annotate(s=f"{cell.id}", xy=cell.polygon.centroid.coords[0], horizontalalignment='center', fontsize=20)

		if adjust_limits:
			x_min, y_min, x_max, y_max = cell.bounds

			self._ax.set_xlim(x_min-1, x_max+1)
			self._ax.set_ylim(y_min-1, y_max+1)


		self._draw()

	def save(self, filename='default.png'):
		self._fig.savefig(filename, bbox_inches='tight', dpi=100)