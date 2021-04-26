import React from 'react';
import { render, screen } from '@testing-library/react';
import NavTabs from '../components/fixed-components/tabs';

describe('Nav Tabs', () => {
    it("should render", () => {
        render(<NavTabs worldName="testWorld" />);
        expect(screen.getByRole('nav-tabs')).toBeVisible();
    });
});
