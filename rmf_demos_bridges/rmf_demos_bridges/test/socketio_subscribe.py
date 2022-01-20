import socketio
import sys

sio = socketio.Client()

topic = '/fleet_states'

try:
    topic = sys.argv[1]
except Exception:
    pass


@sio.event
def connect():
    print('connection established')


@sio.on(topic)
def message(data):
    print(data)


@sio.event
def disconnect():
    print('disconnected from server')


sio.connect('ws://localhost:8080')
sio.wait()
