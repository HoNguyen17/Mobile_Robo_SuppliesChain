# One discrete event inside a run. Maps to a row in the run_events table.

string TYPE_COLLISION=collision
string TYPE_REPLAN=replan
string TYPE_RECOVERY=recovery
string TYPE_WARNING=warning

string event_type                  # one of the TYPE_* constants
float64 sim_time_s                 # seconds since run start
string payload_json                # type-specific detail, e.g. {"other_tag":"NPC"}; "" if none
