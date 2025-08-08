ep = robot.Robot()
ep.initialize(conn_type="ap", ip="192.168.2.1")

print(ep.chassis.get_position())
print(ep.chassis.get_attitude())

ep.close()
