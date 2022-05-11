import sim
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:
    print("Connected to remote API server!")
else:
    print("Failed connecting to remote API server")
sim.simxGetPingTime(clientID)
