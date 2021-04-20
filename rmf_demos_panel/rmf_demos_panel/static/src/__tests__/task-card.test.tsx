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

    const exampleFailedState  = {
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

    const exampleDelayedState  = {
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

    it("should render", () => {
        render(<TaskCard taskState={exampleState} />);
        expect(screen.getByRole('task-details')).toBeInTheDocument();
    });

    it("should render a red X when task has failed", () => {
        const { container } = render(<TaskCard taskState={exampleFailedState} />);
        expect(container.getElementsByClassName("ant-progress-status-exception").length).toBe(1);
    });

    it("should render Delayed description when task is delayed", () => {
        render(<TaskCard taskState={exampleDelayedState} />);
        expect(screen.getByText('Delayed')).toBeInTheDocument();
    });
});
