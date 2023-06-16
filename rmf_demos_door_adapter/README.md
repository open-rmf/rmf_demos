# rmf_demos_door_adapter

Demo door adapter for integration with RMF

## API Endpoints

This door adapter integration relies on a door manager and a door adapter:
- The **door manager** comprises of specific endpoints that help relay commands to the simulated doors. It communicates with the doors over internal ROS 2 messages, while interfacing with the adapter via an API chosen by the user. For this demo door adapter implementation, we are using REST API with FastAPI framework.
- The **door adapter** receives commands from RMF and interfaces with the door manager to receive door state information, as well as query for available doors and send commands to open or close.

To interact with endpoints, launch the demo and then visit http://127.0.0.1:5002/docs in your browser.

### 1. Get Door Names
Get a list of the door names being managed by this adapter. This endpoint does not require a Request Body.

Request URL: `http://127.0.0.1:5002/open-rmf/demo-door/door_names`
##### Response Body:
```json
{
  "data": {
    "door_names": [
      "main_door_left",
      "green_room_door",
      "main_door_right"
    ]
  },
  "success": true,
  "msg": ""
}
```


### 2. Get Door State
Gets the state of the door with the specified name. This endpoint only requires a `door_name` query parameter.

Request URL: `http://127.0.0.1:5002/open-rmf/demo-door/door_state?door_name=door`
##### Response Body:
```json
{
  "data": {
    "current_mode": 0
  },
  "success": true,
  "msg": ""
}
```

### 3. Send Door Request
The `door_request` endpoint allows the door adapter to send requests to a specified door. This endpoint requires a Request Body and a `door_name` query parameter.

Request URL: `http://127.0.0.1:5002/open-rmf/demo-door/door_request?door_name=door`
##### Request Body:
```json
{
  "requested_mode": 0
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
