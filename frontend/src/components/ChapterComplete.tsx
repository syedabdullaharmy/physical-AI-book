import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';

interface ChapterCompleteProps {
    chapterId: string;
}

export default function ChapterComplete({ chapterId }: ChapterCompleteProps) {
    const { user, token, login } = useAuth(); // login actually sets state, we might need a separate updateUser method or just re-fetch
    // Actually simplest is to just re-fetch user in AuthContext if we needed to, or manually update local state.
    // For now, let's assume we just fire the API call. To update UI immediately, we'd need to update the Context user.
    // But our AuthContext as written only fetches on mount/login.

    // Let's rely on local state for the button for now.
    const [isCompleted, setIsCompleted] = useState(false);
    const [loading, setLoading] = useState(false);

    useEffect(() => {
        if (user?.finished_chapters?.includes(chapterId)) {
            setIsCompleted(true);
        }
    }, [user, chapterId]);

    const toggleComplete = async () => {
        if (!user || !token) return alert("Please login to track progress.");
        setLoading(true);

        try {
            const currentChapters = user.finished_chapters || [];
            let newChapters;

            if (isCompleted) {
                newChapters = currentChapters.filter(c => c !== chapterId);
            } else {
                newChapters = [...currentChapters, chapterId];
            }

            const response = await fetch('http://localhost:8000/api/v1/users/me', {
                method: 'PATCH',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${token}`
                },
                body: JSON.stringify({
                    finished_chapters: newChapters
                })
            });

            if (!response.ok) {
                throw new Error("Failed to update progress");
            }

            const updatedUser = await response.json();
            setIsCompleted(!isCompleted);
            // Ideally update global auth context here

        } catch (error) {
            console.error(error);
            alert("Error saving progress");
        } finally {
            setLoading(false);
        }
    };

    return (
        <div className="margin-top--lg p-4 border border-gray-200 dark:border-gray-700 rounded-lg flex items-center justify-between bg-gray-50 dark:bg-gray-800">
            <div>
                <span className="font-bold">Track Progress:</span>
                <span className="ml-2 text-sm text-gray-600 dark:text-gray-400">
                    {isCompleted ? "You have finished this chapter!" : "Mark as read when you are done."}
                </span>
            </div>
            <button
                onClick={toggleComplete}
                disabled={loading}
                className={`px-4 py-2 rounded font-medium ${isCompleted
                        ? "bg-green-100 text-green-800 hover:bg-green-200 border border-green-300"
                        : "bg-indigo-600 text-white hover:bg-indigo-700"
                    } transition-colors`}
            >
                {loading ? "..." : isCompleted ? "✓ Completed" : "Mark Complete"}
            </button>
        </div>
    );
}
