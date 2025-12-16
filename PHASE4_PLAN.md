# Phase 4 Implementation Plan: Auth, Personalization, & Translation

## Overview
This phase adds user accounts, progress tracking, and multi-language support (Urdu) to the Physical AI & Humanoid Robotics Textbook.

## Tasks

### 1. Authentication (Backend & Frontend)
- [ ] **Dependencies**: storage `fastapi-users`, `sqlalchemy`, `alembic` (for migrations if needed).
- [ ] **Backend Setup**:
    - Configure `AsyncSession` with SQLAlchemy.
    - Define `User` model (email, password, is_active, etc.).
    - Implement `fastapi-users` UserManager and Auth Backend (JWT).
    - Add `/auth` endpoints (register, login, logout).
- [ ] **Frontend Setup**:
    - Create `Login` and `Register` pages.
    - Implement `AuthContext` to manage user state.
    - Add "Sign In / Sign Out" button to Navbar.

### 2. Personalization
- [ ] **Database**:
    - Add `UserProfile` or expand `User` model to store:
        - `finished_chapters` (List[str])
        - `bookmarks` (List[str])
        - `settings` (JSON)
- [ ] **API**:
    - `GET /api/v1/users/me`: Get profile.
    - `PATCH /api/v1/users/me`: Update progress/bookmarks.
- [ ] **Frontend**:
    - Track chapter scroll/completion.
    - Show progress bar on dashboard or sidebar.
    - Allow bookmarking sections.

### 3. Urdu Translation (i18n)
- [ ] **Docusaurus Config**:
    - Enable i18n with `locales: ['en', 'ur']`.
    - Configure `localeConfigs` for Urdu (RTL direction).
- [ ] **Content**:
    - Translate a sample chapter (e.g., Chapter 1) to demonstrate capability.
    - Provide a script or workflow to translate other chapters using the RAG/LLM backend.
- [ ] **UI**:
    - Add Language Dropdown to Navbar.
    - Ensure RTL layout works correctly.

## Execution Strategy
1.  **Backend Auth**: Get the API working first (easiest to verify with swaggers).
2.  **Frontend Auth UI**: Connect the UI to the API.
3.  **Personalization**: Add the business logic.
4.  **Translation**: Configure the static site generator.
