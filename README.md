## cb_cpp

A reference implementation of the Constraint-Based Coverage Path Planning framework in python

## Code Example

An example of a possible constraint-based coverage planner.

```python
import robot_primitives as rp

ingress_point = (0.6, 0.6)
egress_point = (9.4, 5.)
sensor_radius = 0.5
vehicle_radius = 0.5

domain = rp.areas.Domain.from_box_corners((0,0), (10,10))

heurisitic = rp.heuristics.EuclideanDistance()
layout = cb_cpp.layouts.SpiralPattern(sensor_radius, vehicle_radius)
sequencer = cb_cpp.sequencers.GreedySequencer(heurisitic)
linker = cb_cpp.linkers.SimpleLinker()

constraints = layout.layout_constraints(domain)
ordered_constraints = sequencer.sequence_constraints(constraints, ingress_point)
path = linker.link_constraints(ordered_constraints)
```

## Installation

To install run:

```
python setup.py install
```
or 
```
python setup.py develop
```