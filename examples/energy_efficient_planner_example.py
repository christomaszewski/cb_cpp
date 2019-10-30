import robot_primitives as rp

from context import cb_cpp

domain_width = 30.
domain_height = 30.
ingress_point = (0., 0.)
#egress_point = (9.4, 5.)
sensor_radius = 0.5
vehicle_radius = 0.5
max_flow_speed = -2.0

domain = rp.areas.Domain.from_box_corners((0,0), (domain_width, domain_height))
#flow_field = rp.fields.VectorField.from_channel_flow_model(channel_width=domain_width, max_velocity=max_flow_speed)
flow_field = rp.fields.VectorField.from_channel_flow_with_pylon(channel_width=domain_width, max_velocity=max_flow_speed, pylon_bounds=(domain_width/2. - 2.5, domain_width/2. + 2.5))

planner = cb_cpp.planners.EnergyEfficientBoustrophedon(vehicle_radius, sensor_radius, flow_field)

num_iterations = 30
path_lengths = set()

for i in range(num_iterations):
	path = planner.plan_coverage_path(domain, ingress_point)
	path_lengths.add(path.length)
	print(path.length)

print(path_lengths)