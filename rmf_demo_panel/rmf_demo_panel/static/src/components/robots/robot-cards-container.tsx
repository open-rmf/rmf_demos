import React from 'react';
import Box from '@material-ui/core/Box';
import Button from '@material-ui/core/Button';
import Grid from '@material-ui/core/Grid';
import Typography from '@material-ui/core/Typography';
import { RobotCard } from './robot-card';
import { useContainerStyles } from '../styles';
import { getRobots } from "../services";
import { socket } from '../socket';

const RobotContainer = () : React.ReactElement => {
    const classes = useContainerStyles();
    const [robotStates, setRobotStates] = React.useState([]);

    const refreshRobotData = async () => {
        let updatedData = await getRobots();
        if(updatedData != robotStates) {
            setRobotStates(updatedData);
        }
    }

    React.useEffect(() => {
        let isSubscribed = true;
        socket.on("robot_states", robotData => {
            if(isSubscribed) {
                setRobotStates(robotData);
            }
        });
        return () => {isSubscribed = false};
    });

    const allRobots = robotStates.map(robotState => {
        return <Grid item><RobotCard robotState={robotState} /></Grid>
    });

    return (
        <Box className={classes.container}>
            <Typography variant="h5">Robots</Typography>
            <Button variant="outlined" onClick={refreshRobotData}>Refresh</Button>
            <Grid container className={classes.grid} >
                {allRobots}
            </Grid>
        </Box>
    )
}
export default RobotContainer;
