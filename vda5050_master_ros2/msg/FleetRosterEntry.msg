# FleetRosterEntry — one AGV declared by the external fleet manager
# (e.g. RMF Device Manager) as part of the desired onboarded set.
#
# Carried inside FleetRoster.msg. Required fields {manufacturer,
# serial_number} are the per-AGV identity used by all master ROS 2
# endpoints (AssignOrder, GetDeviceStatus, etc.).
#
# Optional fields mirror VDA5050Master::onboard_agv()'s C++ defaults:
# leave them at sentinel / default values to accept the master-side
# defaults.

# Required — AGV identity.
string manufacturer
string serial_number

# Optional — per-AGV outbound message queue capacity.
# 0 means "use master default" (currently 10).
uint32 max_queue_size 0

# Optional — queue-full policy for the AGV's outbound publisher.
# true  (default) → drop oldest queued message on overflow.
# false           → reject the new message instead.
bool drop_oldest true
