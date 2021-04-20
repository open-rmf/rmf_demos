import React from 'react';
import MuiAlert, { AlertProps } from '@material-ui/lab/Alert';
import Snackbar from '@material-ui/core/Snackbar';
import IconButton from '@material-ui/core/IconButton';
import CloseIcon from '@material-ui/icons/Close';

function Alert(props: AlertProps) {
  return <MuiAlert elevation={6} variant="filled" {...props} />;
}

export enum NotificationTypes {
  Success = 'success',
  Warning = 'warning',
  Error = 'error',
  Info = 'info'
}

interface SnackbarProps {
  type: NotificationTypes;
  message: string;
  closeSnackbarCallback: () => void;
}

export const NotificationSnackbar = (props: SnackbarProps) => {
  const { type, message, closeSnackbarCallback } = props;
  const [open, setOpen] = React.useState(true);

  const handleClose = (event: React.SyntheticEvent | React.MouseEvent, reason?: string) => {
    if (reason === 'clickaway') {
      return;
    }
    setOpen(false);
    closeSnackbarCallback();
  };

  return (
      <Snackbar
        anchorOrigin={{
          vertical: 'top',
          horizontal: 'center',
        }}
        open={open}
        autoHideDuration={4000}
        onClose={handleClose}
        action={
          <React.Fragment>
            <IconButton size="small" aria-label="close" color="inherit" onClick={handleClose}>
              <CloseIcon fontSize="small" />
            </IconButton>
          </React.Fragment>
        }
      >
        <Alert onClose={handleClose} severity={type}>{message}</Alert>
      </Snackbar>
  )
}