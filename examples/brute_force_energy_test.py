import robot_primitives as rp
import utm

from context import cb_cpp

# Define planning params
sensor_radius = 4.5
vehicle_radius = 1.0
max_velocity = 0.5

# Define domain params
# Lowell Domain #2
#boundary_points = [(42.643525, -71.378496), (42.643886, -71.376763), (42.642238, -71.375848), (42.641831, -71.377781)]
# boundary_points = [(42.643525, -71.378526), (42.643886, -71.376763), (42.642238, -71.375848), (42.641781, -71.377767)]
# center_axis_coords = [(42.643842, -71.377692), (42.641905, -71.376695)]
# ingress_point =  (42.641691, -71.377792)
# egress_point = (42.641691, -71.377792)

# Lowell Short Domain
# boundary_points = [(42.6425789, -71.3781400), (42.6429602, -71.3762517), (42.642238, -71.375848), (42.641781, -71.377767)]
# center_axis_coords = [(42.643842, -71.377692), (42.641905, -71.376695)]
# ingress_point =  (42.641691, -71.377792)
# egress_point = (42.641691, -71.377792)


# Lowell Simple domain
#boundary_points = [(42.6425789, -71.3781400), ( 42.642662, -71.377808 ), (  42.641871, -71.377403  ), (42.641781, -71.377767)]
#boundary_points = [(42.643525, -71.378526), ( 42.643643, -71.378117 ), ( 42.641888, -71.377340 ), (42.641781, -71.377767)]
#center_axis_coords = [(42.643842, -71.377692), (42.641893, -71.376821)] #(42.641905, -71.376695)]   
#ingress_point =  (42.641691, -71.377792)
#egress_point = (42.641691, -71.377792)
#ingress_point = (42.641823333333335, -71.377835)
#egress_point = (42.641823333333335, -71.377835)

 
# Eno domain
# boundary_points = [(36.093018, -78.823576), (36.093226, -78.823354), (36.093178, -78.823197), (36.092943, -78.823392)]
# center_axis_coords = [(36.093230, -78.823244), (36.092956, -78.823512)]  
# ingress_point = (36.092989, -78.823562)
# #ingress_point = (36.093018, -78.823576)
# egress_point = (36.092989, -78.823562)

# Avent domain
# boundary_points = [(35.549243, -79.024726), (35.549941, -79.025559), (35.550199, -79.025197), (35.549529, -79.024404)]
# 3 transect short
#boundary_points = [(35.549332, -79.024603), (35.550037, -79.025490), (35.550199, -79.025197), (35.549529, -79.024404)]
# 4 transect extended
boundary_points = [(35.549332, -79.024603), (35.550136, -79.025583), (35.550300, -79.025311), (35.549529, -79.024404)]
center_axis_coords = [(35.549348, -79.024498), (35.550153, -79.025443)]
ingress_point = (35.550242, -79.025139)
egress_point = (35.550242, -79.025139)


# Convert domain params to utm coords
boundary_utm = [tuple(utm.from_latlon(*pt)[:2]) for pt in boundary_points]
center_axis_utm = [tuple(utm.from_latlon(*pt)[:2]) for pt in center_axis_coords]
ingress_utm = tuple(utm.from_latlon(*ingress_point)[:2])
egress_utm = tuple(utm.from_latlon(*egress_point)[:2])

# Initialize domain and flow field 
domain = rp.areas.Domain.from_vertex_list(boundary_utm)
#flow_field = rp.fields.BoundedVectorField.channel_flow_model(domain, center_axis_utm, max_velocity, undefined_value=(float('nan'),float('nan')))
flow_field  = rp.fields.BoundedVectorField.linear_flow_model(domain, center_axis_utm, 0.0, max_velocity, undefined_value=(float('nan'),float('nan')))

# Initialize all planners
parallel_planner = cb_cpp.planners.ConstraintBasedBoustrophedon.parallel_to_line(vehicle_radius, sensor_radius, center_axis_utm)
ee_planner = cb_cpp.planners.EnergyEfficientBoustrophedon(vehicle_radius, sensor_radius, flow_field, center_axis_utm)
bf_ee_planner = cb_cpp.planners.BruteForceEnergyEfficientBoustrophedon(vehicle_radius, sensor_radius, flow_field, center_axis_utm)

# Plan all paths
print('Planning Parallel Path...')
parallel_path = parallel_planner.plan_coverage_path(domain, ingress_utm)
print('Planning BF EE Path...')
bf_ee_path = bf_ee_planner.plan_coverage_path(domain, ingress_utm, egress_utm)
print('Planning EE Path...')
ee_path = ee_planner.plan_coverage_path(domain, ingress_utm)
print('Done Planning.')

# Add egress point to end of path
parallel_path.add_point(egress_utm)
ee_path.add_point(egress_utm)

print(f"Parallel Path length: {parallel_path.length}")
print(f"BF EE Path length: {bf_ee_path.length}")
print(f"EE Path length: {ee_path.length}")

# Transform paths to lat long coords
#f = lambda pt: utm.to_latlon(pt[0], pt[1], 19, 'T')
f = lambda pt: utm.to_latlon(pt[0], pt[1], 17, 'S')
parallel_path.transform(f)
ee_path.transform(f)
bf_ee_path.transform(f)

domain_prefix = 'avent'

# Save all paths to file
parallel_path.save(f"{domain_prefix}_parallel_path.json")
ee_path.save(f"{domain_prefix}_ee_path.json")
bf_ee_path.save(f"{domain_prefix}_bf_ee_path.json")

#boundary_points = [(42.406071, -71.242046), (42.406071, -71.242608), (42.406347, -71.242608), (42.406347, -71.242046)]
#boundary_utm = [tuple(utm.from_latlon(*pt)[:2]) for pt in boundary_points]
# Pond Domain
#boundary_utm = [(315500., 4697300.), (315450., 4697300.), (315450., 4697330.), (315500., 4697330.)]

# Charles River, Waltham
# Too close to bridge still
#boundary_points = [(42.3689929, -71.2437667), (42.3688012, -71.2432195), (42.368927, -71.243065), (42.369129, -71.243526)]

# Upper charles in front of bridge
#boundary_points = [(42.3683531,-71.2445921), (42.3682835,-71.2441200), (42.3678267, -71.2443470), (42.3679921, -71.2448530)]

# Upper Charles near parking lot launch point
#boundary_points = [(42.362794, -71.245210), (42.362832, -71.245739), (42.363233, -71.245707), (42.363231, -71.245199)]
#boundary_points = [(42.362820, -71.245820), (42.363331, -71.245784), (42.363306, -71.245117), (42.362799, -71.245134)]

#boundary_points = [(42.3628281, -71.2457924), (42.3633236, -71.2457424), (42.363306, -71.245117), (42.362799, -71.245134)]