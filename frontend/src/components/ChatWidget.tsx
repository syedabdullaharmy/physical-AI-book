import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import { useAuth, API_URL } from '../contexts/AuthContext';
import '../css/custom.css';

interface Message {
    role: 'user' | 'assistant' | 'system';
    content: string;
}

export default function ChatWidget() {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState<Message[]>([
        { role: 'assistant', content: 'Hi! I am the Physical AI teaching assistant. Ask me anything about the textbook!' }
    ]);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const messagesEndRef = useRef<HTMLDivElement>(null);
    const { token } = useAuth();

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    };

    useEffect(() => {
        scrollToBottom();
    }, [messages, isOpen]);

    const handleSubmit = async (e: React.FormEvent) => {
        e.preventDefault();
        if (!input.trim() || isLoading) return;

        const userMsg: Message = { role: 'user', content: input };
        setMessages(prev => [...prev, userMsg]);
        setInput('');
        setIsLoading(true);

        try {
            // Connect to backend API
            const headers: Record<string, string> = { 'Content-Type': 'application/json' };
            if (token) {
                headers['Authorization'] = `Bearer ${token}`;
            }

            const response = await fetch(`${API_URL}/chat`, {
                method: 'POST',
                headers: headers,
                body: JSON.stringify({
                    message: userMsg.content,
                    history: messages.filter(m => m.role !== 'system')
                }),
            });

            if (!response.ok) throw new Error('Network response was not ok');

            const reader = response.body?.getReader();
            const decoder = new TextDecoder();
            let assistantMsg: Message = { role: 'assistant', content: '' };

            setMessages(prev => [...prev, assistantMsg]);

            while (true) {
                const { done, value } = await reader!.read();
                if (done) break;

                const text = decoder.decode(value);
                assistantMsg.content += text;

                // Update last message with new partial content
                setMessages(prev => [
                    ...prev.slice(0, -1),
                    { ...assistantMsg }
                ]);
            }

        } catch (error) {
            setMessages(prev => [...prev, { role: 'system', content: 'Error: Could not connect to AI backend. Please ensure the backend server is running.' }]);
        } finally {
            setIsLoading(false);
        }
    };

    // Chat Icon SVG
    const ChatIcon = () => (
        <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
        </svg>
    );

    // Close Icon SVG
    const CloseIcon = () => (
        <svg xmlns="http://www.w3.org/2000/svg" width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
        </svg>
    );

    // Send Icon SVG
    const SendIcon = () => (
        <svg xmlns="http://www.w3.org/2000/svg" width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <line x1="22" y1="2" x2="11" y2="13"></line>
            <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
        </svg>
    );

    return (
        <>
            {/* Chat Button */}
            {!isOpen && (
                <button
                    onClick={() => setIsOpen(true)}
                    className="chat-widget-button"
                    aria-label="Open Chat"
                >
                    <ChatIcon />
                </button>
            )}

            {/* Chat Window */}
            {isOpen && (
                <div className="chat-window">
                    <div className="chat-header">
                        <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
                            <ChatIcon />
                            <span>AI Assistant</span>
                        </div>
                        <button
                            onClick={() => setIsOpen(false)}
                            className="clean-btn"
                            style={{ color: 'white', display: 'flex', alignItems: 'center' }}
                            aria-label="Close Chat"
                        >
                            <CloseIcon />
                        </button>
                    </div>

                    <div className="chat-body">
                        {messages.map((msg, idx) => (
                            <div key={idx} className={clsx('message', {
                                'message-user': msg.role === 'user',
                                'message-assistant': msg.role === 'assistant',
                                'message-system': msg.role === 'system'
                            })}>
                                {msg.content}
                            </div>
                        ))}
                        {isLoading && (
                            <div className="message message-assistant" style={{ fontStyle: 'italic', color: 'var(--ifm-color-primary)' }}>
                                <span className="typing-indicator">Thinking...</span>
                            </div>
                        )}
                        <div ref={messagesEndRef} />
                    </div>

                    <div className="chat-footer">
                        <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '8px', alignItems: 'center' }}>
                            <input
                                type="text"
                                value={input}
                                onChange={(e) => setInput(e.target.value)}
                                placeholder="Ask about robotics..."
                                className="chat-input"
                            />
                            <button
                                type="submit"
                                disabled={isLoading}
                                className="button button--primary button--sm"
                                style={{ borderRadius: '8px', padding: '0.6rem', display: 'flex', alignItems: 'center', justifyContent: 'center' }}
                            >
                                <SendIcon />
                            </button>
                        </form>
                    </div>
                </div>
            )}
        </>
    );
}
