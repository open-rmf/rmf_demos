import React from "react";
import AppBar from '@material-ui/core/AppBar';
import Toolbar from '@material-ui/core/Toolbar';
import Typography from '@material-ui/core/Typography';

const Header = () : React.ReactElement => {
    return (
        <div role="header">
            <AppBar id="appbar" position="sticky">
                <Toolbar>
                    <Typography variant="h4">RMF Panel</Typography>
                </Toolbar>
            </AppBar>
        </div>
    )
}

export default Header;