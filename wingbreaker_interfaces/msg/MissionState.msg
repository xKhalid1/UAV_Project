# State published by the interceptor mission supervisor.
uint8 IDLE=0
uint8 CONNECTING=1
uint8 PATROL=2
uint8 LOCK=3        # intruder detected, locking on (chase + orbit)
uint8 DECISION=4    # awaiting interception approval (human or LLM)
uint8 ENGAGE=5      # firing / intercepting
uint8 REPORT=6      # publishing intercept report, returning to patrol
uint8 ERROR=255

std_msgs/Header header
uint8 state
string detail
