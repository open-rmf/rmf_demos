import React from "react";
import Typography from '@material-ui/core/Typography';
import { useFooterStyles } from "../styles";

const Footer = () : React.ReactElement => {
    const classes = useFooterStyles();
    
    return (
        <footer className={classes.footer} role="footer">
            <Typography variant="subtitle1">Developed by Open Robotics 2020</Typography>
            <Typography variant="subtitle1">RMF Demo Panel is Powered by RMF</Typography>
            <img src="https://static1.squarespace.com/static/57daacd7f7e0ab084d5efbf0/t/5e3a197c40f86549694b35a7/1604624644022/?format=1500w" alt="osrf_logo" width="80" height="30"/>
        </footer>
    )
}

export default Footer;