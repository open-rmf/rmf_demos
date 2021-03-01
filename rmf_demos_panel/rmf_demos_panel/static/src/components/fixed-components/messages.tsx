import { message } from 'antd';

export const showSuccessMessage = (displayString: string) => {
    const config = {
        content: displayString,
        duration: 4,
        style: { marginTop: '10vh'}
    }
    message.success(config);
}

export const showErrorMessage = (displayError: string) => {
    const config = {
        content: displayError,
        duration: 4,
        style: { marginTop: '10vh'}
    }
    message.error(config);
}