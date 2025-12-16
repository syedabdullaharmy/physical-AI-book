import React, { createContext, useContext, useState, useEffect } from 'react';

// API Configuration
export const API_URL = (typeof process !== 'undefined' && process.env?.NEXT_PUBLIC_API_URL) || 'http://localhost:8000/api/v1';

interface User {
    id: string;
    email: string;
    first_name?: string;
    last_name?: string;
    bookmarks?: string[];
    finished_chapters?: string[];
}

interface AuthContextType {
    user: User | null;
    token: string | null;
    loading: boolean;
    login: (token: string) => void;
    logout: () => void;
}

const AuthContext = createContext<AuthContextType>({
    user: null,
    token: null,
    loading: true,
    login: () => { },
    logout: () => { },
});

export const useAuth = () => useContext(AuthContext);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
    const [token, setToken] = useState<string | null>(null);
    const [user, setUser] = useState<User | null>(null);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        // Check if we are in browser environment
        if (typeof window !== 'undefined') {
            const savedToken = localStorage.getItem('auth_token');
            if (savedToken) {
                setToken(savedToken);
                // Don't fetch user on initial load to prevent hanging
                // User will be fetched after successful login
                setLoading(false);
            } else {
                setLoading(false);
            }
        } else {
            setLoading(false);
        }
    }, []);

    const fetchUser = async (authToken: string) => {
        try {
            // Add timeout to prevent hanging
            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), 5000); // 5 second timeout

            const response = await fetch(`${API_URL}/users/me`, {
                headers: { Authorization: `Bearer ${authToken}` },
                signal: controller.signal
            });

            clearTimeout(timeoutId);

            if (response.ok) {
                const userData = await response.json();
                setUser(userData);
            } else {
                console.error("Failed to fetch user, logging out");
                logout();
            }
        } catch (error) {
            console.error("Error fetching user:", error);
            logout();
        } finally {
            setLoading(false);
        }
    };

    const login = (newToken: string) => {
        localStorage.setItem('auth_token', newToken);
        setToken(newToken);
        setLoading(true);
        fetchUser(newToken);
    };

    const logout = () => {
        localStorage.removeItem('auth_token');
        setToken(null);
        setUser(null);
    };

    return (
        <AuthContext.Provider value={{ user, token, loading, login, logout }}>
            {children}
        </AuthContext.Provider>
    );
};
