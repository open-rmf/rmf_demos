import React from 'react';
import { fireEvent, render, screen, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event'; 
import ScheduledTaskForm from '../components/forms/scheduled-task-form';

describe('Scheduled Task Form', () => {
    let root: ReturnType<typeof renderForm>;
    let submitRequest = jest.fn();

    function renderForm() {
        return render(<ScheduledTaskForm submitTaskList={submitRequest} />);
    }

    beforeEach(() => {
        root = renderForm();
    });

    test("should render", () => {
        expect(screen.getByRole('scheduled-task-form')).toBeInTheDocument();
    });

    test("should render error messages when empty form is submitted", () => {
        const submitButton = screen.getByText('Submit Task List');
        fireEvent.click(submitButton);
        expect(submitRequest).not.toHaveBeenCalled();
    });

    test("should submit a valid form", () => {
        const tasks = `[ {"task_type":"Clean", "start_time":0, "description": {"cleaning_zone":"zone_1"}}, {"task_type":"Clean", "start_time":10, "description": {"cleaning_zone":"zone_2"}}, {"task_type":"Clean", "start_time":5, "description": {"cleaning_zone":"zone_3"}} ]`
        const submitButton = screen.getByText('Submit Task List');
        const taskListBox = root.getByPlaceholderText(/eg.*/);
        userEvent.type(taskListBox, tasks);
        fireEvent.click(submitButton);
        expect(submitRequest).toHaveBeenCalled();
    });
});
