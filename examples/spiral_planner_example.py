import robot_primitives as rp

from context import cb_cpp

ingress_point = (0.6, 0.6)
egress_point = (9.4, 5.)
sensor_radius = 0.5
vehicle_radius = 0.5

domain = rp.areas.Domain.from_box_corners((0,0), (10,10))

planner = cb_cpp.planners.ConstraintBasedSpiral(vehicle_radius, sensor_radius)
path = planner.plan_coverage_path(domain, ingress_point)

print(path.length)