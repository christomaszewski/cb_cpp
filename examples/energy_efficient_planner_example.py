import robot_primitives as rp

from context import cb_cpp

domain_width = 10.
ingress_point = (0.6, 0.6)
egress_point = (9.4, 5.)
sensor_radius = 0.5
vehicle_radius = 0.5
max_flow_speed = -2.0

domain = rp.areas.Domain.from_box_corners((0,0), (domain_width, domain_width))
flow_field = rp.fields.VectorField.from_channel_flow_model(channel_width=domain_width, max_velocity=max_flow_speed)

planner = cb_cpp.planners.EnergyEfficientBoustrophedon(sensor_radius, vehicle_radius, flow_field)
path = planner.plan_coverage_path(domain, ingress_point)

print(path.length)