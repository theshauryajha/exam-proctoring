# Ground-truth state of one simulated student.

string id                        # "r<row>_c<col>"; row 0 is nearest the whiteboard
uint8 row
uint8 col
geometry_msgs/Point seat         # chair centre in the world (odom) frame

string behavior                  # current episode, e.g. "writing", "phone", "passing_note"
string phase                     # step within the episode, e.g. "reaching", "handing_over"
bool is_cheating                 # the episode is a cheating episode
bool cheating_visible            # the current phase shows the cheating action or prop
string partner_id                # other student involved (copied from, note partner); empty if none
builtin_interfaces/Time behavior_start
