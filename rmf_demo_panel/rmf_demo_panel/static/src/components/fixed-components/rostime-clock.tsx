import React from 'react';
import Box from '@material-ui/core/Box';
import Typography from '@material-ui/core/Typography';
import { socket } from '../socket';

const RostimeClock = () => {
    const [time, setTime] = React.useState('');

    React.useEffect(() => {
        let isSubscribed = true;
        socket.on("ros_time", msg => {
            if(isSubscribed) {
                const timeData = Math.floor(msg / 3600) + ":" +
                (Math.floor(msg / 60) % 60) + ":" +
                (msg % 60);
                setTime(timeData);
            }
        });
        return () => {isSubscribed = false}
    }, []);

    return (
        <Box>
            <Typography variant='h6' align="center" gutterBottom role="clock-time">Time is: {time}</Typography>
        </Box>
    )
}

export default RostimeClock;
