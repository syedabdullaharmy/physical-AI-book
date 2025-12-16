import React from 'react';
import ChapterToolbar from './ChapterToolbar';

interface MDXChapterProps {
    chapterId: string;
    chapterTitle: string;
    children: React.ReactNode;
}

export default function MDXChapter({ chapterId, chapterTitle, children }: MDXChapterProps) {
    return (
        <div className="mdx-chapter-wrapper">
            <ChapterToolbar chapterId={chapterId} chapterTitle={chapterTitle} />
            <div className="mdx-chapter-content">
                {children}
            </div>
        </div>
    );
}
