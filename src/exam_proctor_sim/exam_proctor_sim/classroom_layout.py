"""Single source of truth for the classroom geometry.

The world generator, the patrol node and the student behaviour node all import
this module, so desks, aisles and student ids always agree.

World frame: origin at the room centre on the floor, +x points to the front
of the room (whiteboard), students sit facing +x. Desks come in side-by-side
pairs; the robot drives in the aisles between pairs and between rows.
"""

ROOM_X = 14.0           # front-to-back inner length [m]
ROOM_Y = 13.0           # side-to-side inner width [m]
WALL_H = 3.0
WALL_T = 0.15

ROWS = 4
PAIRS = 3               # desk pairs per row
COLS = 2 * PAIRS        # seats per row
ROW_PITCH = 2.4         # desk centre spacing along x
PAIR_PITCH = 3.6        # desk-pair centre spacing along y
FRONT_ROW_X = 2.5       # x of the front row desk centres

DESK_DEPTH = 0.6        # along x
DESK_WIDTH = 1.2        # along y; paired desks touch, so neighbours sit 1.2 m apart
DESK_HEIGHT = 0.75
CHAIR_OFFSET = 0.55     # chair centre sits this far behind (-x) the desk centre
CHAIR_SIZE = 0.45

TEACHER_DESK_X = 5.2
TEACHER_DESK_DEPTH = 0.8
TEACHER_DESK_WIDTH = 1.6

ROBOT_RADIUS = 0.22
MIN_CLEARANCE = 0.3     # free space required on each side of the robot


def desk_x(row):
    return FRONT_ROW_X - row * ROW_PITCH


def desk_y(col):
    pair, seat = divmod(col, 2)
    return (pair - (PAIRS - 1) / 2.0) * PAIR_PITCH + (seat - 0.5) * DESK_WIDTH


def desks():
    """[(row, col, x, y)] of every student desk centre."""
    return [(r, c, desk_x(r), desk_y(c)) for r in range(ROWS) for c in range(COLS)]


def student_id(row, col):
    return f'r{row}_c{col}'


def seat_position(row, col):
    """(x, y) of the chair centre, where the student model is placed."""
    return desk_x(row) - CHAIR_OFFSET, desk_y(col)


def neighbour_col(col):
    """Column of the student sharing the desk pair."""
    return col + 1 if col % 2 == 0 else col - 1


def row_extent(row):
    """(back_x, front_x) occupied by a row of desks plus chairs."""
    return (desk_x(row) - CHAIR_OFFSET - CHAIR_SIZE / 2.0,
            desk_x(row) + DESK_DEPTH / 2.0)


def corridor_xs():
    """x of the cross corridors (running along y), front to back."""
    front = (row_extent(0)[1] + TEACHER_DESK_X - TEACHER_DESK_DEPTH / 2.0) / 2.0
    between = [(row_extent(r)[0] + row_extent(r + 1)[1]) / 2.0 for r in range(ROWS - 1)]
    back = (row_extent(ROWS - 1)[0] - ROOM_X / 2.0) / 2.0
    return [front] + between + [back]


def aisle_ys():
    """y of the aisles (running along x), right to left."""
    outer = (desk_y(COLS - 1) + DESK_WIDTH / 2.0 + ROOM_Y / 2.0) / 2.0
    between = [(p - (PAIRS - 1) / 2.0 + 0.5) * PAIR_PITCH for p in range(PAIRS - 1)]
    return [-outer] + between + [outer]


def check_clearance():
    """Raise if any aisle is too narrow for the robot."""
    need = 2 * (ROBOT_RADIUS + MIN_CLEARANCE)
    widths = [
        TEACHER_DESK_X - TEACHER_DESK_DEPTH / 2.0 - row_extent(0)[1],
        *[row_extent(r)[0] - row_extent(r + 1)[1] for r in range(ROWS - 1)],
        row_extent(ROWS - 1)[0] + ROOM_X / 2.0,
        PAIR_PITCH - 2 * DESK_WIDTH,
        ROOM_Y / 2.0 - desk_y(COLS - 1) - DESK_WIDTH / 2.0,
    ]
    if min(widths) < need:
        raise ValueError(f'aisle of {min(widths):.2f} m is narrower than {need:.2f} m')
