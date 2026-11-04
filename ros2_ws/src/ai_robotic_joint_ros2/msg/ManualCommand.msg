# A sequence identifies an operator action; repeated packets are heartbeats.
uint8 STOP=0
uint8 MOVE=1
uint8 KEYBOARD=1
uint8 ANGLE=2
uint8 AI_PLACEHOLDER=3
uint64 sequence
uint8 mode
uint8 action
float64 target_angle_rad
