import React from 'react';
import Card from '@material-ui/core/Card';
import CardContent from '@material-ui/core/CardContent';
import Grid from '@material-ui/core/Grid';
import Typography from '@material-ui/core/Typography';
import Progress from 'antd/lib/progress';
import { useRobotCardStyles } from '../styles';
import BatteryChargingFullIcon from '@material-ui/icons/BatteryChargingFull';
import Dock from '@material-ui/icons/Dock';

interface RobotCardProps {
  robotState: {
    robot_name: string;
    fleet_name: string;
    assignments: string[];
    mode: string;
    battery_percent: string;
    level_name: string;
  };
}

export const RobotCard = (props: RobotCardProps) : React.ReactElement => {
    const { robotState } = props
    let battery: number = parseFloat(parseFloat(robotState.battery_percent).toFixed(2));
    const classes = useRobotCardStyles();

    const returnStatusIcon = (mode: string) => {
      switch(mode) {
        case "Charging-1":
          return <BatteryChargingFullIcon />
        case "Dock/Clean-7":
          return <Dock />
      }
    }

    const joinAssignments = (assignments: string[]) => {
      if(assignments.length > 1) {
        let joinedAssignments = assignments.map((value, index) => {
          if(index === (assignments.length - 1)){
            return value;
          } else {
            return value.concat(" ➜")
          }
        });
        return joinedAssignments.join("");
      } else if (assignments.length == 1) {
        return assignments[0];
      } else {
        return '';
      }
    }

    return (
        <Card className={classes.root} variant="outlined" role="robot-details">
            <CardContent>
              <Grid container>
                <Grid item xs={12}>
                    <Typography variant="h6" color="textSecondary" className={classes.text}>
                      {robotState.robot_name}
                    </Typography>
                  </Grid>
                  <Grid item xs={12}>
                    <Typography variant="subtitle1" color="textSecondary" className={classes.text}>
                      {robotState.fleet_name}
                    </Typography>
                  </Grid>
                  <Grid item xs={5}>
                    <Typography variant="subtitle2" color="textSecondary" gutterBottom>
                      Assigned Tasks
                    </Typography>
                  </Grid>
                  <Grid item xs={7}><Typography>{joinAssignments(robotState.assignments)}</Typography></Grid>
                  <Grid item xs={5}>
                    <Typography variant="subtitle2" color="textSecondary" gutterBottom>
                      Status
                    </Typography>
                  </Grid>
                  <Grid item xs={7}><Typography>{robotState.mode}</Typography></Grid>
                  <Grid item xs={5}>
                    <Typography variant="subtitle2" color="textSecondary" gutterBottom>
                      Battery
                    </Typography>
                  </Grid>
                  <Grid item xs={6}>
                    <Progress percent={battery} size="small" width={50}/>
                  </Grid>
                  <Grid item xs={5}>
                    <Typography variant="subtitle2" color="textSecondary">
                      Location
                    </Typography>
                  </Grid>
                  <Grid item xs={7}><Typography>{robotState.level_name}</Typography></Grid>
                  <Grid item container direction="column" alignItems="flex-end" justify="flex-end">{returnStatusIcon(robotState.mode)}</Grid>
              </Grid>
            </CardContent>
        </Card>
    );                                                                
}
