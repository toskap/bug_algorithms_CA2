from controller import Supervisor
import math

MAX_COORDS = 200       
REFRESH_FACTOR = 10    

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep()) * REFRESH_FACTOR

epuck_node = supervisor.getFromDef('EPUCK')
if epuck_node is None:
    print("Error: No node with DEF 'EPUCK' found!")
    exit(1)

existing_trail = supervisor.getFromDef('TRAIL')
if existing_trail:
    existing_trail.remove()

trail_string = 'DEF TRAIL Shape {\n'
trail_string += '  appearance Appearance {\n'
trail_string += '    material Material {\n'
trail_string += '      diffuseColor 0 1 0\n'
trail_string += '      emissiveColor 0 1 0\n'
trail_string += '    }\n'
trail_string += '  }\n'
trail_string += '  geometry DEF TRAIL_LINE_SET IndexedLineSet {\n'
trail_string += '    coord Coordinate {\n'
trail_string += '      point [\n'
for _ in range(MAX_COORDS):
    trail_string += '        0 0 0\n'
trail_string += '      ]\n'
trail_string += '    }\n'
trail_string += '    coordIndex [\n'
for _ in range(MAX_COORDS):
    trail_string += '      0 0 -1\n'
trail_string += '    ]\n'
trail_string += '  }\n'
trail_string += '}\n'

root_children_field = supervisor.getRoot().getField('children')
root_children_field.importMFNodeFromString(-1, trail_string)

trail_node = supervisor.getFromDef('TRAIL_LINE_SET')
coord_node = trail_node.getField('coord').getSFNode()
point_field = coord_node.getField('point')
coord_index_field = trail_node.getField('coordIndex')

index = 0
first_step = True

while supervisor.step(timestep) != -1:
    # pos = epuck_node.getPosition()

    # point_field.setMFVec3f(index, pos)

    # if index > 0:
        # coord_index_field.setMFInt32(3 * (index - 1), index - 1)
        # coord_index_field.setMFInt32(3 * (index - 1) + 1, index)
    # elif index == 0 and not first_step:
        # coord_index_field.setMFInt32(3 * (MAX_COORDS - 1), 0)
        # coord_index_field.setMFInt32(3 * (MAX_COORDS - 1) + 1, MAX_COORDS - 1)

    # coord_index_field.setMFInt32(3 * index, index)
    # coord_index_field.setMFInt32(3 * index + 1, index)

    # first_step = False
    # index += 1
    # index %= MAX_COORDS
    continue