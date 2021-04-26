import React from 'react';
import { cleanup, fireEvent, render, screen, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event'; 
import CleaningForm from '../components/forms/cleaning-form';

describe('Cleaning Form', () => {
    let root: ReturnType<typeof renderForm>;
    let submitRequest = jest.fn();
    let setTimeError = jest.fn();
    let setMinsFromNow = jest.fn();
    let setPriority = jest.fn();
    let setPriorityError = jest.fn();
    let minsFromNow = 0;
    let priority = 0;
    let timeAndPriority = { minsFromNow, priority, setTimeError, setMinsFromNow, setPriority, setPriorityError }

    function renderForm() {
        const cleaningZones = ['zone1', 'zone2'];
        return render(<CleaningForm cleaningZones={cleaningZones} submitRequest={submitRequest} timeAndPriority={timeAndPriority} />);
    }

    beforeEach(() => {
        root = renderForm();
    });

    afterEach(() => {
        root.unmount();
        cleanup();
    });

    it("should render", () => {
        expect(screen.getByRole('cleaning-form')).toBeInTheDocument();
    });

    it("should render error messages when invalid form is submitted", () => {
        const submitButton = screen.getByText('Submit Request');
        fireEvent.click(submitButton);
        expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
        expect(screen.getByText("Cleaning zone cannot be an empty field")).toBeInTheDocument();
    });

    it("should submit a valid form", () => {
        const submitButton = screen.getByText('Submit Request');
        userEvent.click(root.getByLabelText('Pick a zone'));
        userEvent.click(within(screen.getAllByRole('listbox')[0]).getByText('zone1'));
        userEvent.click(submitButton);
        expect(submitRequest).toHaveBeenCalled();
    });
});
