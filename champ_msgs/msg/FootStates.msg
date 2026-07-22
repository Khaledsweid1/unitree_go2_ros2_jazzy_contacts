# Per-foot ground-truth contact state + reaction force, aggregated from the
# four Gazebo foot contact sensors. All arrays are length 4 and index-aligned
# with `names` (default order: lf, rf, lh, rh).
std_msgs/Header header
string[] names                 # foot names, e.g. ['lf', 'rf', 'lh', 'rh']
bool[] in_contact              # true if the foot is touching something
geometry_msgs/Vector3[] force  # ground-reaction force on the foot, world frame [N]
float64[] normal_force         # convenience: |force.z| per foot [N]
