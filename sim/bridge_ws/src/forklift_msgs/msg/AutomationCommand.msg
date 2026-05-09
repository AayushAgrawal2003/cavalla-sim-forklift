# Wraps a ForkliftDirectCommand with task tracking metadata so the orchestrator
# and a higher-level scheduler can correlate commands to the task that issued
# them. Every automation source topic (and /orchestrator/active_task) uses this.

# Status enum
uint8 STATUS_IDLE      = 0
uint8 STATUS_RUNNING   = 1
uint8 STATUS_COMPLETED = 2
uint8 STATUS_FAILED    = 3
uint8 STATUS_ABORTED   = 4

string command_id        # uuid issued by orchestrator; "" when source is idle / no task
string source            # publisher node name (e.g. "fork_height", "line_follower")
string action            # task type the source thinks it's running (e.g. "fork_height")
uint8  status            # one of STATUS_*
string detail            # optional human-readable detail / completion reason

ForkliftDirectCommand cmd
