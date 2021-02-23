import React from 'react';
import { cleanup, fireEvent, render, screen, waitFor } from '@testing-library/react';
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

    afterEach(() => {
        root.unmount();
        cleanup();
    })

    test("should render", () => {
        expect(screen.getByRole('scheduled-task-form')).toBeInTheDocument();
    });

    test("should render error messages when empty form is submitted", () => {
        const submitButton = screen.getByText('Submit Task List');
        fireEvent.click(submitButton);
        expect(root.getByText("Unable to submit an empty task list")).toBeTruthy();
        expect(submitRequest).not.toHaveBeenCalled();
    });

    test("should show the file name and text content when a file is uploaded", async () => {
        const tasks = new File([`[{"task_type":"Clean", "start_time":0, "description": {"cleaning_zone":"zone_1"}}]`], 'tasks.json', {
            type: 'json',
        });
        
        const inputEl = root.getByLabelText('Select File');
        Object.defineProperty(inputEl, 'files', {
            value: [tasks],
        });

        await waitFor(() => {
            fireEvent.change(inputEl);
            expect(root.getByText('[{"task_type":"Clean", "start_time":0, "description": {"cleaning_zone":"zone_1"}}]')).toBeTruthy();
            expect(root.queryByLabelText('Select File')).toBeNull();
            expect(root.getByLabelText('tasks.json')).toBeTruthy();
        });
    });

    test("should submit a valid form", () => {
        const tasks = `[ {"task_type":"Clean", "start_time":0, "description": {"cleaning_zone":"zone_1"}}, {"task_type":"Clean", "start_time":10, "description": {"cleaning_zone":"zone_2"}}, {"task_type":"Clean", "start_time":5, "description": {"cleaning_zone":"zone_3"}} ]`
        const submitButton = screen.getByText('Submit Task List');
        const taskListBox = root.getByPlaceholderText(/eg.*/);
        userEvent.type(taskListBox, tasks);
        fireEvent.click(submitButton);
        expect(submitRequest).toHaveBeenCalled();
    });

    test("should render text content when the same file is uploaded again after first submission", async () => {
        const list = `[{"task_type":"Clean", "start_time":0, "description": {"cleaning_zone":"zone_1"}}]`
        const tasks = new File([list], 'tasks.json', {
            type: 'json',
        });
        
        const inputEl = root.getByLabelText('Select File');
        Object.defineProperty(inputEl, 'files', {
            value: [tasks],
        });

        await waitFor(() => {
            fireEvent.change(inputEl);
            expect(root.getByText(list)).toBeTruthy();
            expect(root.queryByLabelText('Select File')).toBeNull();
            expect(root.getByLabelText('tasks.json')).toBeTruthy();
        });
        const submitButton = root.getByText('Submit Task List');
        fireEvent.click(submitButton);
        expect(submitRequest).toHaveBeenCalled();
        
        await waitFor(() => {
            expect(root.queryByText(list)).toBeNull();
        });

        await waitFor(() => {
            fireEvent.change(inputEl);
            expect(root.getByText(list)).toBeTruthy();
            expect(root.queryByLabelText('Select File')).toBeNull();
            expect(root.getByLabelText('tasks.json')).toBeTruthy();
        });
    });
});
