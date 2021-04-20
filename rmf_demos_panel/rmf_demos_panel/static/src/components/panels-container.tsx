import React from "react"
import Divider from '@material-ui/core/Divider';
import Grid from '@material-ui/core/Grid';
import Typography from '@material-ui/core/Typography';
import ScheduledTaskForm from './forms/scheduled-task-form';
import RobotContainer from './robots/robot-cards-container';
import TasksContainer from './tasks/tasks-container';
import RostimeClock from './fixed-components/rostime-clock';
import RequestForm from './forms/request-form';
import { usePanelContainerStyles } from "./styles";
import { submitAllTasks } from './services';

const PanelsContainer = (): React.ReactElement => {
    const classes = usePanelContainerStyles();

    return (
            <Grid className={classes.panels}>
                <Grid container direction="row" alignContent="center" justify="center">
                    <Grid item xs={12}>
                        <RostimeClock />
                    </Grid>
                    <Grid item xs={12}>
                        <Typography className={classes.centered} variant="h4">Task Submissions</Typography>
                    </Grid>
                </Grid>
                <Grid container direction="row" justify="space-evenly" alignItems="flex-start" spacing={2}>
                    <Grid item xs={5}>
                        <RequestForm />
                    </Grid>
                    <Grid item xs={5}>
                        <ScheduledTaskForm submitTaskList={submitAllTasks} />
                    </Grid>
                </Grid>
                <Divider variant="middle"/>
                <Grid container direction="row" alignContent="center" justify="center">
                    <Grid item xs={12}>
                        <Typography className={classes.centered} variant="h4">Robots & Tasks Summaries</Typography>
                    </Grid>
                </Grid>
                 <Grid container direction="row" justify="space-evenly" alignItems="flex-start" spacing={2}>
                    <Grid item xs={5}>
                        <RobotContainer />
                    </Grid>
                    <Grid item xs={6}>
                        <TasksContainer />
                    </Grid>
                </Grid>
            </Grid>
    )
}

export default PanelsContainer;