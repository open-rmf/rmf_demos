import React from 'react';
import Button from '@material-ui/core/Button';
import Card from '@material-ui/core/Card';
import CardContent from '@material-ui/core/CardContent';
import Grid from '@material-ui/core/Grid';
import Typography from '@material-ui/core/Typography';
import Progress from 'antd/lib/progress';
import StyledChip from '../styled-components/styled-chip';
import { useTaskCardStyles } from '../styles';
import { cancelTask } from '../services';
import { orange } from '@material-ui/core/colors';
import { NotificationSnackbar, NotificationTypes } from '../fixed-components/notification-snackbar';

interface TaskCardProps {
    taskState: {
        task_id: string,
        description: string,
        robot_name: string,
        state: 	string,
        task_type: string,
        priority: number,
        start_time: number,
        end_time: number,
        progress: string,
    }
}

export const TaskCard = (props: TaskCardProps) : React.ReactElement => {
  const { taskState } = props;
  const [lastKnownProgress, setLastKnownProgress] = React.useState(0);
  const [currentTaskStatus, setCurrentTaskStatus] = React.useState(taskState.state);
  const [snackbarMessage, setSnackbarMessage] = React.useState('');
  const [openSnackbar, setOpenSnackbar] = React.useState(false);
  const [messageType, setMessageType] = React.useState(NotificationTypes.Success);
  const classes = useTaskCardStyles();

  const returnProgressBar = (type: string) => {
    switch (type) {
      case 'Cancelled':
        return (<Progress type="dashboard" gapDegree={120} showInfo={false}/>);

      case 'Delayed':
        return (<Progress type="dashboard" strokeColor={orange[50]} gapDegree={120} percent={lastKnownProgress}/>);

      case 'Failed':
        return (<Progress type="dashboard" gapDegree={120} percent={lastKnownProgress} status="exception" />);

      default:
        return (<Progress type="dashboard" gapDegree={120} percent={lastKnownProgress} />);
    }
  }

  React.useEffect(() => {
    let progValue = parseInt(taskState.progress);
    if(isNaN(parseInt(taskState.progress))) {
      setCurrentTaskStatus('Delayed');
    } else {
      setCurrentTaskStatus(taskState.state);
      setLastKnownProgress(progValue);
    }
  }, [taskState.progress, taskState.state]);

  const showErrorSnackbar = (message: string): void => {
    setSnackbarMessage(message);
    setMessageType(NotificationTypes.Error);
    setOpenSnackbar(true);
  }

  const showSuccessSnackbar = (message: string): void => {
    setSnackbarMessage(message);
    setMessageType(NotificationTypes.Success);
    setOpenSnackbar(true);
  }

  const handleCancel = async (id: string) => {
    let response = await cancelTask(id);
    if (response["success"] === false) {
      showErrorSnackbar(`Failed to cancel Task ID [${id}]`);
    } else {
      setCurrentTaskStatus('Cancelled');
      showSuccessSnackbar(`Task ID [${id}] cancelled successfully`);
    }
  }

  return (
      <Card className={classes.root} variant="outlined" role="task-details">
          <CardContent>
            <Grid container>
              <Grid item xs={12}>
                {returnProgressBar(currentTaskStatus)}
              </Grid>
                <Grid item xs={6}>
                  <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                    Task ID
                  </Typography>
                </Grid>
                <Grid item xs={6}><Typography>{taskState.task_id}</Typography></Grid>
                <Grid item xs={6}>
                  <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                    Details
                  </Typography>
                </Grid>
                <Grid item xs={6}><Typography>{taskState.description}</Typography></Grid>
                <Grid item xs={6}>
                  <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                    Robot
                  </Typography>
                </Grid>
                <Grid item xs={6}><Typography>{taskState.robot_name}</Typography></Grid>
                <Grid item xs={6}>
                  <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                    Task Type
                  </Typography>
                </Grid>
                <Grid item xs={6}><Typography>{taskState.task_type}</Typography></Grid>
                <Grid item xs={6}>
                  <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                    Priority
                  </Typography>
                </Grid>
                <Grid item xs={6}><Typography>{taskState.priority}</Typography></Grid>
                <Grid item xs={6}>
                  <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                    Task State
                  </Typography>
                </Grid>
                <Grid item xs={6}>
                  <StyledChip state={taskState.state}/>
                </Grid>
                <Grid item xs={3}>
                  <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                    Start:
                  </Typography>
                </Grid>
                <Grid item xs={3}><Typography>{taskState.start_time}</Typography></Grid>
                <Grid item xs={3}>
                  <Typography variant="subtitle2" align="left" color="textSecondary" gutterBottom>
                    End:
                  </Typography>
                </Grid>
                <Grid item xs={3}><Typography>{taskState.end_time}</Typography></Grid>
                <Grid item xs={12}><Button variant="outlined" onClick={() => handleCancel(taskState.task_id)} disabled={ ['Cancelled', 'Completed', 'Failed'].indexOf(currentTaskStatus) != -1 }>Cancel Task</Button></Grid>
            </Grid>
          </CardContent>
          {openSnackbar && <NotificationSnackbar type={messageType} message={snackbarMessage} closeSnackbarCallback={() => setOpenSnackbar(false)} />}
      </Card>
  );
}
