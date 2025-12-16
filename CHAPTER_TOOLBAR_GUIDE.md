# Adding Translation & Personalization to Chapters

## Overview

The ChapterToolbar component provides translation to Urdu and personalization features for logged-in users. This guide shows how to add it to your chapters.

## Quick Start

### Method 1: Using MDX Import (Recommended)

Add this to the top of any chapter markdown file:

```mdx
---
sidebar_position: 1
---

import ChapterToolbar from '@site/src/components/ChapterToolbar';

<ChapterToolbar 
    chapterId="module-1/chapter-1" 
    chapterTitle="ROS 2 Architecture and Core Concepts" 
/>

# Chapter 1: ROS 2 Architecture and Core Concepts

[Rest of your content...]
```

### Method 2: Global Integration (All Chapters)

To add the toolbar to ALL chapters automatically, swizzle the DocItem component:

```bash
cd frontend
npm run swizzle @docusaurus/theme-classic DocItem/Layout -- --wrap
```

Then edit `src/theme/DocItem/Layout/index.tsx`:

```tsx
import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import ChapterToolbar from '@site/src/components/ChapterToolbar';
import { useDoc } from '@docusaurus/theme-common/internal';

export default function LayoutWrapper(props) {
  const { metadata } = useDoc();
  
  // Extract chapter ID from frontmatter or path
  const chapterId = metadata.frontMatter.chapterId || metadata.id;
  const chapterTitle = metadata.title;
  
  return (
    <>
      <ChapterToolbar chapterId={chapterId} chapterTitle={chapterTitle} />
      <Layout {...props} />
    </>
  );
}
```

## Features

### 1. Translate to Urdu Button

- **Icon**: Translation icon with Urdu text "اردو میں پڑھیں"
- **Functionality**: 
  - Calls `/api/v1/translate-to-urdu` endpoint
  - Caches translation for future use
  - Displays content in RTL format
  - Preserves code blocks and technical terms

**User Experience:**
1. User clicks "اردو میں پڑھیں" button
2. Loading spinner shows "Translating..."
3. Translation appears below toolbar
4. Toggle between Original/Urdu views

### 2. Personalize for Me Button

- **Icon**: User profile icon
- **Functionality**:
  - Calls `/api/v1/personalize-content` endpoint
  - Adapts content to user's background
  - Adjusts difficulty level
  - Customizes examples

**User Experience:**
1. User clicks "Personalize for Me" button
2. Loading spinner shows "Personalizing..."
3. Personalized content appears
4. Toggle between Original/Personalized views

### 3. View Toggle

Once content is translated or personalized, users can toggle between:
- **Original**: Default chapter content
- **Urdu**: Translated version
- **Personalized**: Adapted to user's level

### 4. Login Prompt

For non-logged-in users:
- Shows "Login to translate and personalize content" link
- Redirects to `/login` page
- After login, returns to chapter

## Backend API

### Translation Endpoint

```python
POST /api/v1/translate-to-urdu
Content-Type: application/json
Authorization: Bearer {token}

{
    "chapter_id": "module-1/chapter-1",
    "user_id": "user123"
}

Response:
{
    "content": "# اردو ترجمہ\n\n...",
    "cached": false
}
```

### Personalization Endpoint

```python
POST /api/v1/personalize-content
Content-Type: application/json
Authorization: Bearer {token}

{
    "chapter_id": "module-1/chapter-1",
    "user_id": "user123"
}

Response:
{
    "content": "# Personalized Content for You\n\n...",
    "cached": false
}
```

### Cache Management

```python
DELETE /api/v1/clear-cache/{chapter_id}

Response:
{
    "chapter_id": "module-1/chapter-1",
    "cleared": ["translation", "personalization (3 users)"],
    "message": "Cleared cache for module-1/chapter-1"
}
```

## Styling

The toolbar uses modern, premium design:

- **Glassmorphism effects**
- **Smooth animations**
- **Gradient backgrounds**
- **Hover effects**
- **Dark mode support**
- **Responsive design**

### Customization

Edit `ChapterToolbar.module.css` to customize:

```css
/* Change button colors */
.translateButton {
    background: linear-gradient(135deg, your-color-1, your-color-2);
}

/* Adjust spacing */
.chapterToolbar {
    margin-bottom: 3rem; /* Increase space */
}

/* Modify animations */
.toolbarButton:hover {
    transform: translateY(-3px); /* More lift */
}
```

## Example Chapter with Toolbar

```mdx
---
sidebar_position: 1
chapterId: module-1/chapter-1
---

import ChapterToolbar from '@site/src/components/ChapterToolbar';

<ChapterToolbar 
    chapterId="module-1/chapter-1" 
    chapterTitle="ROS 2 Architecture and Core Concepts" 
/>

# Chapter 1: ROS 2 Architecture and Core Concepts

## Introduction

Welcome to your journey into the world of robotics middleware!

[Rest of content...]
```

## Testing

### Test Translation

1. Login to the application
2. Navigate to any chapter
3. Click "اردو میں پڑھیں" button
4. Verify translation appears
5. Toggle between Original and Urdu views

### Test Personalization

1. Login with a user profile
2. Navigate to any chapter
3. Click "Personalize for Me" button
4. Verify personalized content appears
5. Check content matches user's level

### Test Caching

1. Translate a chapter
2. Refresh the page
3. Translate again - should be instant (cached)
4. Check backend logs for cache hit

## Troubleshooting

### Button Not Showing

**Issue**: Toolbar doesn't appear
**Solution**: 
- Check import statement in MDX
- Verify component path is correct
- Ensure AuthContext is wrapped around app

### Translation Fails

**Issue**: "Translation failed" error
**Solution**:
- Check backend is running
- Verify API URL in AuthContext
- Check network tab for errors
- Ensure user is logged in

### Styling Issues

**Issue**: Buttons look broken
**Solution**:
- Verify CSS module is imported
- Check for CSS conflicts
- Clear browser cache
- Check dark mode styles

## Performance Optimization

### Caching Strategy

- **Translation Cache**: Shared across all users
- **Personalization Cache**: Per-user caching
- **Cache Invalidation**: Clear when chapter updated

### Lazy Loading

Consider lazy loading translations:

```tsx
const [translatedContent, setTranslatedContent] = useState<string | null>(null);

// Only load when button clicked
const handleTranslate = async () => {
    if (!translatedContent) {
        const content = await fetchTranslation();
        setTranslatedContent(content);
    }
};
```

## Future Enhancements

Potential improvements:

1. **Offline Support**: Cache translations in localStorage
2. **Multiple Languages**: Add more language options
3. **Voice Reading**: Text-to-speech for accessibility
4. **Export**: Download personalized/translated content
5. **Share**: Share personalized versions with others

## Files Created

```
frontend/src/components/
├── ChapterToolbar.tsx           # Main toolbar component
├── ChapterToolbar.module.css    # Toolbar styles
└── MDXChapter.tsx              # MDX wrapper component

backend/app/api/endpoints/
└── content.py                   # Translation & personalization API
```

## Summary

The ChapterToolbar provides:
- ✅ Translation to Urdu for all users
- ✅ Personalization based on user profile
- ✅ Caching for performance
- ✅ Beautiful, modern UI
- ✅ Responsive design
- ✅ Dark mode support

Users can now enjoy content in their preferred language and difficulty level!
