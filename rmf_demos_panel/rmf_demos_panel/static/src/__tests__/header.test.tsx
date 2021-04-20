import React from 'react';
import { render, screen } from '@testing-library/react';
import Header from '../components/fixed-components/header';

describe('Header', () => {
    it("should render", () => {
        render(<Header />);
        expect(screen.getByRole('header')).toBeVisible();
    
    });
})