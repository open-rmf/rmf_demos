import socketio

sio = socketio.Client()


@sio.event
def connect():
  print('connection established')


@sio.on('tinyRobot')
def tinyRobot_fleet_state(data):
  print(data)


@sio.event
def disconnect():
  print('disconnected from server')


sio.connect('ws://localhost:8080')
sio.wait()
