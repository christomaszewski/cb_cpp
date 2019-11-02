import robot_primitives as rp
import numpy as np

import matplotlib.pyplot as plt

from context import cb_cpp

ingress_point = (0., 0.)
egress_point = (10., 10.)
sensor_radius = 0.75
vehicle_radius = 0.5

#domain = rp.areas.Domain.from_box_corners((0,0), (11,11))
domain  = rp.areas.Domain.from_vertex_list([(5.,0.), (10.,5.), (5.,10.), (0.,5.)])
target_side = domain.get_side(3)
print(f"Planning paths with transects parallel and perpendicular to line {target_side}")

parallel_planner = cb_cpp.planners.ConstraintBasedBoustrophedon.parallel_to_side(vehicle_radius, sensor_radius, target_side)
perp_planner = cb_cpp.planners.ConstraintBasedBoustrophedon.perpendicular_to_side(vehicle_radius, sensor_radius, target_side)

parallel_path = parallel_planner.plan_coverage_path(domain)
perp_path = perp_planner.plan_coverage_path(domain)

print(f"Parallel Boustrophedon path length: {parallel_path.length}")
print(f"Perpendicular Boustrophedon path length: {perp_path.length}")

parallel_coords = np.array(parallel_path.coord_list)
perp_coords = np.array(perp_path.coord_list)

fig = plt.figure(figsize=(20, 10), dpi=100)
mean_center = np.mean(perp_coords, axis=0)

ax = fig.add_subplot(1,2,1)
ax.plot(perp_coords[:,0], perp_coords[:,1], '-g')
ax = fig.add_subplot(1,2,2)
ax.plot(parallel_coords[:,0], parallel_coords[:,1], '-b')
plt.show()

# angled_planner = cb_cpp.planners.ConstraintBasedBoustrophedon.parallel_to_side(vehicle_radius, sensor_radius, domain.get_side(0))
# vertical_planner = cb_cpp.planners.ConstraintBasedBoustrophedon.vertical(vehicle_radius, sensor_radius)
# horizontal_planner = cb_cpp.planners.ConstraintBasedBoustrophedon.horizontal(vehicle_radius, sensor_radius)

# vertical_path = vertical_planner.plan_coverage_path(domain)
# horizontal_path = horizontal_planner.plan_coverage_path(domain)
# angled_path = angled_planner.plan_coverage_path(domain)

# vertical_path.save('vertical.json')
# horizontal_path.save('horizontal.json')
# angled_path.save('angled.json')

# print(f"Vertical Boustrophedon path length: {vertical_path.length}")
# print(f"Horizontal Boustrophedon path length: {horizontal_path.length}")
# print(f"Angled Boustrophedon path length: {angled_path.length}")

# coords = np.array(horizontal_path.coord_list)
# angled_coords = np.array(angled_path.coord_list)
# vert_coords = np.array(vertical_path.coord_list)

# fig = plt.figure(figsize=(12, 12), dpi=100)
# mean_center = np.mean(coords, axis=0)

# ax = fig.add_subplot(1,1,1)
# extent_buffer = 10.0
# bottom_left = mean_center - extent_buffer
# top_right = mean_center + extent_buffer
# extents = [bottom_left[0], top_right[0], bottom_left[1], top_right[1]]
# #ax.set_extents(extents)
# plt.plot(coords[:,0], coords[:,1], '-g')
# plt.plot(angled_coords[:,0], angled_coords[:,1], '-b')
# plt.plot(vert_coords[:,0], vert_coords[:,1], '-r')
# plt.show()

# plt.savefig('horizontal.png')