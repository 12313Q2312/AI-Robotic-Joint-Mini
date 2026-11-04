# One supervised command owns the shared reference for both arms.
ManualCommand control
uint8 payload_index
# Changes only while stopped; acknowledged after both physical resets verify.
uint64 reset_sequence
