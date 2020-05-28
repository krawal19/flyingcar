from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False)
drone = Drone(conn,tlog_name="TLog-manual.txt")
drone.start()
drone.stop()