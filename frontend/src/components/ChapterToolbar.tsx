import React, { useState } from 'react';
import { useAuth, API_URL } from '../contexts/AuthContext';
import styles from './ChapterToolbar.module.css';

interface ChapterToolbarProps {
    chapterId: string;
    chapterTitle: string;
}

export default function ChapterToolbar({ chapterId, chapterTitle }: ChapterToolbarProps) {
    const { user, token } = useAuth();
    const [isTranslating, setIsTranslating] = useState(false);
    const [isPersonalizing, setIsPersonalizing] = useState(false);
    const [translatedContent, setTranslatedContent] = useState<string | null>(null);
    const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
    const [showTranslated, setShowTranslated] = useState(false);
    const [showPersonalized, setShowPersonalized] = useState(false);
    const [error, setError] = useState<string | null>(null);

    const handleTranslate = async () => {
        if (!user || !token) {
            setError('Please login to translate content');
            return;
        }

        setIsTranslating(true);
        setError(null);

        try {
            const response = await fetch(`${API_URL}/translate-to-urdu`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${token}`
                },
                body: JSON.stringify({
                    chapter_id: chapterId,
                    user_id: user.id
                })
            });

            if (!response.ok) {
                throw new Error('Translation failed');
            }

            const data = await response.json();
            setTranslatedContent(data.content);
            setShowTranslated(true);
        } catch (err) {
            setError('Failed to translate content. Please try again.');
            console.error('Translation error:', err);
        } finally {
            setIsTranslating(false);
        }
    };

    const handlePersonalize = async () => {
        if (!user || !token) {
            setError('Please login to personalize content');
            return;
        }

        setIsPersonalizing(true);
        setError(null);

        try {
            const response = await fetch(`${API_URL}/personalize-content`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${token}`
                },
                body: JSON.stringify({
                    chapter_id: chapterId,
                    user_id: user.id
                })
            });

            if (!response.ok) {
                throw new Error('Personalization failed');
            }

            const data = await response.json();
            setPersonalizedContent(data.content);
            setShowPersonalized(true);
        } catch (err) {
            setError('Failed to personalize content. Please try again.');
            console.error('Personalization error:', err);
        } finally {
            setIsPersonalizing(false);
        }
    };

    const toggleView = (type: 'original' | 'translated' | 'personalized') => {
        if (type === 'translated') {
            setShowTranslated(!showTranslated);
            setShowPersonalized(false);
        } else if (type === 'personalized') {
            setShowPersonalized(!showPersonalized);
            setShowTranslated(false);
        } else {
            setShowTranslated(false);
            setShowPersonalized(false);
        }
    };

    return (
        <div className={styles.chapterToolbar}>
            <div className={styles.toolbarContent}>
                <div className={styles.toolbarTitle}>
                    <h1>{chapterTitle}</h1>
                </div>

                {user ? (
                    <div className={styles.toolbarActions}>
                        {/* Translate Button */}
                        <button
                            onClick={handleTranslate}
                            disabled={isTranslating}
                            className={`${styles.toolbarButton} ${styles.translateButton} ${showTranslated ? styles.active : ''}`}
                            title="Translate to Urdu"
                        >
                            {isTranslating ? (
                                <>
                                    <span className={styles.spinner}></span>
                                    <span>Translating...</span>
                                </>
                            ) : (
                                <>
                                    <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor">
                                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M3 5h12M9 3v2m1.048 9.5A18.022 18.022 0 016.412 9m6.088 9h7M11 21l5-10 5 10M12.751 5C11.783 10.77 8.07 15.61 3 18.129" />
                                    </svg>
                                    <span>اردو میں پڑھیں</span>
                                </>
                            )}
                        </button>

                        {/* Personalize Button */}
                        <button
                            onClick={handlePersonalize}
                            disabled={isPersonalizing}
                            className={`${styles.toolbarButton} ${styles.personalizeButton} ${showPersonalized ? styles.active : ''}`}
                            title="Personalize for your level"
                        >
                            {isPersonalizing ? (
                                <>
                                    <span className={styles.spinner}></span>
                                    <span>Personalizing...</span>
                                </>
                            ) : (
                                <>
                                    <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor">
                                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M16 7a4 4 0 11-8 0 4 4 0 018 0zM12 14a7 7 0 00-7 7h14a7 7 0 00-7-7z" />
                                    </svg>
                                    <span>Personalize for Me</span>
                                </>
                            )}
                        </button>

                        {/* View Toggle */}
                        {(translatedContent || personalizedContent) && (
                            <div className={styles.viewToggle}>
                                <button
                                    onClick={() => toggleView('original')}
                                    className={`${styles.toggleButton} ${!showTranslated && !showPersonalized ? styles.active : ''}`}
                                >
                                    Original
                                </button>
                                {translatedContent && (
                                    <button
                                        onClick={() => toggleView('translated')}
                                        className={`${styles.toggleButton} ${showTranslated ? styles.active : ''}`}
                                    >
                                        Urdu
                                    </button>
                                )}
                                {personalizedContent && (
                                    <button
                                        onClick={() => toggleView('personalized')}
                                        className={`${styles.toggleButton} ${showPersonalized ? styles.active : ''}`}
                                    >
                                        Personalized
                                    </button>
                                )}
                            </div>
                        )}
                    </div>
                ) : (
                    <div className={styles.loginPrompt}>
                        <a href="/book/login" className={styles.loginLink}>
                            Login to translate and personalize content
                        </a>
                    </div>
                )}
            </div>

            {error && (
                <div className={styles.errorMessage}>
                    <svg className={styles.errorIcon} viewBox="0 0 24 24" fill="none" stroke="currentColor">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4m0 4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                    </svg>
                    <span>{error}</span>
                </div>
            )}

            {/* Render translated or personalized content */}
            {showTranslated && translatedContent && (
                <div className={styles.alternativeContent} dir="rtl">
                    <div dangerouslySetInnerHTML={{ __html: translatedContent }} />
                </div>
            )}

            {showPersonalized && personalizedContent && (
                <div className={styles.alternativeContent}>
                    <div dangerouslySetInnerHTML={{ __html: personalizedContent }} />
                </div>
            )}
        </div>
    );
}
