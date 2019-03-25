import robot_primitives as rp

from context import cb_cpp

ingress_point = (0.6, 0.6)
egress_point = (9.4, 5.)
sensor_radius = 0.5
vehicle_radius = 0.5

domain = rp.areas.Domain.from_box_corners((0,0), (11,11))

vertical_planner = cb_cpp.planners.ConstraintBasedBoustrophedon(sensor_radius, vehicle_radius)
horizontal_planner = cb_cpp.planners.ConstraintBasedBoustrophedon(sensor_radius, vehicle_radius, horizontal=True)
vertical_path = vertical_planner.plan_coverage_path(domain, ingress_point)
horizontal_path = horizontal_planner.plan_coverage_path(domain, ingress_point)

print(f"Vertical Boustrophedon path length: {vertical_path.length}")
print(f"Horizontal Boustrophedon path length: {horizontal_path.length}")