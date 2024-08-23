from pymavlink import mavutil

time_step = 1 / 20

connection = mavutil.mavlink_connection("udpin:localhost:14569")

connection.wait_heartbeat()

print("Heartbeat for system (system %u component %u)" % (connection.target_system, connection.target_component))