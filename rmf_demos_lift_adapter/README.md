# roscon_lift_adapter

Lift adapter for the roscon workshop

## API Endpoints

This lift adapter integration relies on a lift manager and a lift adapter:
- The **lift manager** comprises of specific endpoints that help relay commands to the simulated lifts. It communicates with the lifts over internal ROS 2 messages, while interfacing with the adapter via an API chosen by the user. For this demo lift adapter implementation, we are using REST API with FastAPI framework.
- The **lift adapter** receives commands from RMF and interfaces with the lift manager to receive lift state information, as well as query for available lifts and send commands to go to different floors and open / close doors.

To interact with endpoints, launch the demo and then visit http://127.0.0.1:5003/docs in your browser.

### 1. Get Lift Names
Get a list of the lift names being managed by this adapter. This endpoint does not require a Request Body.

Request URL: `http://127.0.0.1:5003/open-rmf/demo-lift/lift_names`
##### Response Body:
```json
{
  "data": {
    "lift_names": [
      "lift"
    ]
  },
  "success": true,
  "msg": ""
}
```


### 2. Get Lift State
Gets the state of the lift with the specified name. This endpoint only requires a `lift_name` query parameter.

Request URL: `http://127.0.0.1:5003/open-rmf/demo-lift/lift_state?lift_name=lift`
##### Response Body:
```json
{
  "data": {
    "available_floors": [
      "L1",
      "L2"
    ],
    "current_floor": "L1",
    "destination_floor": "L1",
    "door_state": 0,
    "motion_state": 0
  },
  "success": true,
  "msg": ""
}
```

### 3. Send Lift Request
The `lift_request` endpoint allows the lift adapter to send requests to a specified lift. This endpoint requires a Request Body and a `lift_name` query parameter.

Request URL: `http://127.0.0.1:5003/open-rmf/demo-lift/lift_request?lift_name=lift`
##### Request Body:
```json
{
  "floor": "L2",
  "door_state": 0
}
```

##### Response Body:
```json
{
  "data": {},
  "success": true,
  "msg": ""
}
```
