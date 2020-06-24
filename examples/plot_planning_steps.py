import sys
import numpy as np
from pathlib import Path

import robot_primitives as rp
import robot_utils as rut

from context import cb_cpp

scenario_name = sys.argv[1]
scenario_dir = Path(scenario_name)
domain_filename = str(scenario_dir / 'domain.json')
domain = rp.areas.Domain.from_file(domain_filename)

vehicle_radius = 1.0
sensor_radius = 10.

domain_ingress_point = None
domain_egress_point = None

dv = rut.plotting.DomainView(domain, title='Test Scenario')
dv.plot_domain()

# Vertical transects
transect_orientation = (0., 1.)

"""
heuristic = rp.heuristics.DirectedDistance.perpendicular(transect_orientation)
layout = cb_cpp.layouts.OrientedBoustrophedonPattern.from_transect_orientation(vehicle_radius, sensor_radius, transect_orientation)
refinements = [cb_cpp.refinements.AlternatingDirections()]
sequencer = cb_cpp.sequencers.GreedySequencer(heuristic)
linker = cb_cpp.linkers.SimpleLinker()
"""
heuristic = rp.heuristics.DirectedDistance.perpendicular(transect_orientation)
layout = cb_cpp.layouts.OrientedBoustrophedonPattern.from_transect_orientation(vehicle_radius, sensor_radius, transect_orientation)
refinements = [cb_cpp.refinements.AlternatingDirections()]
sequencer = cb_cpp.sequencers.GreedySequencer(heuristic)
linker = cb_cpp.linkers.SimpleLinker()

constraints = layout.layout_constraints(domain)
dv.plot_constraints(constraints, plot_sequence=False)
dv.save(f"{scenario_dir}/output/raw_constraints.png")
dv.clear_figure()
dv.plot_domain()

for r in refinements:
	r.refine_constraints(constraints, area_ingress_point=domain_ingress_point, starting_direction=[0, 1])

dv.plot_constraints(constraints, plot_sequence=False)
dv.save(f"{scenario_dir}/output/directed_constraints.png")
dv.clear_figure()
dv.plot_domain()

constraint_chain = sequencer.sequence_constraints(constraints, domain_ingress_point)

dv.plot_constraints(constraints, plot_sequence=True)
dv.save(f"{scenario_dir}/output/sequenced_constraints.png")
dv.clear_figure()
dv.plot_domain()

path = linker.link_constraints(constraint_chain, domain_ingress_point)

dv.plot_path(path, color='xkcd:melon')
dv.save(f"{scenario_dir}/output/path.png")