# High-level autonomous mission state published by Rust navigation FSM
std_msgs/Header header

uint8 STATE_IDLE = 0
uint8 STATE_SEARCHING_TARGET = 1
uint8 STATE_APPROACHING_TARGET = 2
uint8 STATE_ALIGNING_FOR_PICKUP = 3
uint8 STATE_PICKING_UP = 4
uint8 STATE_SEARCHING_DELIVERY_ZONE = 5
uint8 STATE_NAVIGATING_TO_DELIVERY = 6
uint8 STATE_RELEASING_OBJECT = 7
uint8 STATE_COMPLETED = 8
uint8 STATE_FAULT = 99

uint8 state
string state_name
float64 distance_to_goal_m
bool holding_object
