import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import ChatWidget from '../components/ChatWidget';

// Wrapper for the entire application to provide Authentication context and global components
export default function Root({ children }: { children: React.ReactNode }) {
    return (
        <AuthProvider>
            {children}
            <ChatWidget />
        </AuthProvider>
    );
}
