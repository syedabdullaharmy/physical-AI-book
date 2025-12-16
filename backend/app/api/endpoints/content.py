"""
Content personalization and translation endpoints.
"""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Optional
import os

router = APIRouter()


class TranslateRequest(BaseModel):
    chapter_id: str
    user_id: str


class PersonalizeRequest(BaseModel):
    chapter_id: str
    user_id: str


class ContentResponse(BaseModel):
    content: str
    cached: bool = False


# In-memory cache for demo (use Redis in production)
translation_cache = {}
personalization_cache = {}


def get_chapter_content(chapter_id: str) -> str:
    """
    Get original chapter content from markdown files.
    
    Args:
        chapter_id: Chapter identifier (e.g., "module-1/chapter-1")
    
    Returns:
        Chapter content as markdown string
    """
    # Construct path to chapter file
    base_path = os.path.join(os.path.dirname(__file__), "../../../../frontend/docs")
    chapter_path = os.path.join(base_path, f"{chapter_id}.md")
    
    try:
        with open(chapter_path, 'r', encoding='utf-8') as f:
            return f.read()
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail=f"Chapter {chapter_id} not found")


def translate_to_urdu(content: str) -> str:
    """
    Translate content to Urdu using Mistral AI.
    
    This is a placeholder. In production, integrate with Mistral AI API.
    
    Args:
        content: Original English content
    
    Returns:
        Translated Urdu content
    """
    # TODO: Integrate with Mistral AI API
    # For now, return a demo translation
    
    # Simulate translation by adding Urdu prefix
    translated = f"""
# اردو ترجمہ

:::info
یہ خودکار ترجمہ ہے۔ تکنیکی اصطلاحات انگریزی میں محفوظ ہیں۔
:::

{content}

---
**نوٹ**: یہ ایک ڈیمو ترجمہ ہے۔ مکمل ترجمے کے لیے Mistral AI API کو مربوط کریں۔
"""
    return translated


def personalize_content(content: str, user_id: str) -> str:
    """
    Personalize content based on user profile.
    
    This is a placeholder. In production, integrate with Mistral AI API
    and user profile data.
    
    Args:
        content: Original content
        user_id: User identifier
    
    Returns:
        Personalized content
    """
    # TODO: Get user profile from database
    # TODO: Integrate with Mistral AI API for personalization
    
    # For now, return demo personalized content
    personalized = f"""
# Personalized Content for You

:::tip Your Learning Path
This content has been adapted to your experience level and interests.
:::

{content}

---
**Note**: This is a demo. Full personalization requires Mistral AI API integration and user profile data.
"""
    return personalized


@router.post("/translate-to-urdu", response_model=ContentResponse)
async def translate_chapter(request: TranslateRequest):
    """
    Translate chapter content to Urdu.
    
    This endpoint:
    1. Checks cache for existing translation
    2. If not cached, gets original content
    3. Translates using Mistral AI
    4. Caches the result
    5. Returns translated content
    """
    cache_key = f"{request.chapter_id}:ur"
    
    # Check cache
    if cache_key in translation_cache:
        return ContentResponse(
            content=translation_cache[cache_key],
            cached=True
        )
    
    # Get original content
    try:
        original_content = get_chapter_content(request.chapter_id)
    except HTTPException as e:
        raise e
    
    # Translate content
    translated_content = translate_to_urdu(original_content)
    
    # Cache translation
    translation_cache[cache_key] = translated_content
    
    return ContentResponse(
        content=translated_content,
        cached=False
    )


@router.post("/personalize-content", response_model=ContentResponse)
async def personalize_chapter(request: PersonalizeRequest):
    """
    Personalize chapter content based on user profile.
    
    This endpoint:
    1. Checks cache for existing personalized content
    2. If not cached, gets original content
    3. Gets user profile from database
    4. Personalizes using Mistral AI
    5. Caches the result
    6. Returns personalized content
    """
    cache_key = f"{request.chapter_id}:user:{request.user_id}"
    
    # Check cache
    if cache_key in personalization_cache:
        return ContentResponse(
            content=personalization_cache[cache_key],
            cached=True
        )
    
    # Get original content
    try:
        original_content = get_chapter_content(request.chapter_id)
    except HTTPException as e:
        raise e
    
    # Personalize content
    personalized_content = personalize_content(original_content, request.user_id)
    
    # Cache personalization
    personalization_cache[cache_key] = personalized_content
    
    return ContentResponse(
        content=personalized_content,
        cached=False
    )


@router.delete("/clear-cache/{chapter_id}")
async def clear_chapter_cache(chapter_id: str):
    """
    Clear cached translations and personalizations for a chapter.
    
    Useful when chapter content is updated.
    """
    cleared = []
    
    # Clear translation cache
    translation_key = f"{chapter_id}:ur"
    if translation_key in translation_cache:
        del translation_cache[translation_key]
        cleared.append("translation")
    
    # Clear personalization cache (all users)
    keys_to_delete = [k for k in personalization_cache.keys() if k.startswith(f"{chapter_id}:user:")]
    for key in keys_to_delete:
        del personalization_cache[key]
    if keys_to_delete:
        cleared.append(f"personalization ({len(keys_to_delete)} users)")
    
    return {
        "chapter_id": chapter_id,
        "cleared": cleared,
        "message": f"Cleared cache for {chapter_id}"
    }
