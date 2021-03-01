import React from 'react';
import {  makeStyles } from '@material-ui/core/styles';
import Chip from '@material-ui/core/Chip';
import { red, orange, blue, teal, grey, purple, cyan } from '@material-ui/core/colors';

interface StyledChipProps {
    state: string
}

const useStyles = (bgCol: string) => makeStyles({
    root: {
        background: bgCol,
        color: 'white',
        height: 24,
        verticalAlign: 'middle',
    }
});

const returnChipColor = (state: string): string => {
    switch(state) {
        case "Queued": 
            return cyan[500]
        case "Active/Executing":
            return blue[500]
        case "Completed":
            return teal[500]
        case "Failed":
            return red[400]
        case "Cancelled":
            return grey[400]
        case "Pending":
            return purple[400]
        case "Delayed":
            return orange[300]
    }
}

const StyledChip = (props: StyledChipProps) => {
    const { state } = props;
    const classes = useStyles(returnChipColor(state))();

    return (
        <Chip
            classes={{
                root: classes.root
            }}
            label={state}
            size="small"
        />
    );
}

export default StyledChip;