import React from 'react';
import { render, screen } from '@testing-library/react';
import { TaskCard } from '../components/tasks/task-card';

describe('Task Card', () => {
    const exampleState  = {
        task_id: "1003",
        description: "zone_x",
        robot_name: "magnus",
        state: "Active/Executing",
        task_type: "Clean",
        priority: 1,
        start_time: 1500,
        end_time: 1540,
        progress: "42"
    };

    const failedState  = {
        task_id: "1003",
        description: "zone_x",
        robot_name: "magnus",
        state: "Failed",
        task_type: "Clean",
        priority: 1,
        start_time: 1500,
        end_time: 1540,
        progress: "42"
    };

    const delayedState  = {
        task_id: "1003",
        description: "zone_x",
        robot_name: "magnus",
        state: "Delayed",
        task_type: "Clean",
        priority: 1,
        start_time: 1500,
        end_time: 1540,
        progress: "42"
    };

    const cancelledState  = {
        task_id: "1003",
        description: "zone_x",
        robot_name: "magnus",
        state: "Cancelled",
        task_type: "Clean",
        priority: 1,
        start_time: 1500,
        end_time: 1540,
        progress: "42"
    };

    it("should render", () => {
        render(<TaskCard taskState={exampleState} />);
        expect(screen.getByRole('task-details')).toBeInTheDocument();
    });

    it("should render a red X when task has failed", () => {
        const { container } = render(<TaskCard taskState={failedState} />);
        expect(container.getElementsByClassName("ant-progress-status-exception").length).toBe(1);
    });

    it("should render Delayed description when task is delayed", () => {
        render(<TaskCard taskState={delayedState} />);
        expect(screen.getByText('Delayed')).toBeInTheDocument();
    });

    it("should render Cancelled state with disabled cancel button when task is cancelled", () => {
        render(<TaskCard taskState={cancelledState} />);
        expect(screen.getByText('Cancelled')).toBeInTheDocument();
        expect(screen.getByRole('button', { name: 'Cancel Task'})).toBeDisabled();
    });
});
