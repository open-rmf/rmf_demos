import io from 'socket.io-client';

const ENDPOINT = "http://" + location.hostname + ":8080/status_updates"

export const socket = io.connect(ENDPOINT);