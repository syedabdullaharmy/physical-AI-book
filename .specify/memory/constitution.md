# Project Constitution: Physical AI & Humanoid Robotics Textbook

## Project Identity

**Name:** Physical AI & Humanoid Robotics Interactive Textbook  
**Purpose:** Create a comprehensive, interactive textbook with RAG chatbot for teaching Physical AI & Humanoid Robotics course  
**Target Audience:** Students learning embodied AI, robotics, and humanoid systems

## Core Principles

### 1. Educational Excellence
- Content must be technically accurate and pedagogically sound
- Complex concepts explained progressively from fundamentals to advanced
- Include practical examples, code snippets, and real-world applications
- Support multiple learning levels (beginner to advanced)

### 2. Technology Stack Standards

#### Frontend
- **Framework:** Docusaurus (latest stable version)
- **Language:** TypeScript for type safety
- **Styling:** Tailwind CSS with custom theme
- **Components:** React functional components with hooks
- **State Management:** React Context API for global state

#### Backend
- **Framework:** FastAPI (Python 3.9+)
- **Database:** Neon Serverless Postgres
- **Vector Storage:** Qdrant Cloud Free Tier
- **AI Model:** Mistral AI API (MANDATORY - NOT OpenAI)
- **Authentication:** Better-Auth

#### Deployment
- **Frontend:** GitHub Pages
- **Backend:** Vercel or Railway
- **CI/CD:** GitHub Actions

### 3. Code Quality Standards

#### All Code Must:
- Follow language-specific best practices (PEP 8 for Python, Airbnb style for JavaScript/TypeScript)
- Include comprehensive error handling with specific error messages
- Have logging at appropriate levels (DEBUG, INFO, WARNING, ERROR)
- Be type-annotated (TypeScript types, Python type hints)
- Include JSDoc/docstring comments for all public functions
- Pass linting (ESLint, Pylint) with no errors

#### Testing Requirements:
- Unit tests for all business logic (>80% coverage)
- Integration tests for API endpoints
- E2E tests for critical user flows
- Test files co-located with source files

### 4. Performance Requirements
- **Page Load:** < 3 seconds (initial load)
- **Time to Interactive:** < 5 seconds
- **API Response:** < 500ms (95th percentile)
- **RAG Query:** < 3 seconds for complete response
- **Mobile Responsive:** All features work on mobile devices

### 5. Security Standards
- **Never** commit API keys, secrets, or credentials
- All sensitive data in environment variables
- Use `.env.example` with placeholder values
- Implement rate limiting on all API endpoints
- Sanitize all user inputs
- Use HTTPS for all production endpoints
- Implement proper CORS configuration

### 6. Architecture Principles

#### Separation of Concerns:
- Frontend handles only presentation and user interaction
- Backend handles business logic, data access, and AI integration
- Clear API contracts between layers

#### Modularity:
- Components should be small, focused, and reusable
- Services should have single responsibility
- Avoid tight coupling between modules

#### Scalability:
- Design for horizontal scaling (stateless services)
- Use caching strategically (Redis for frequent queries)
- Optimize database queries with proper indexing

### 7. Documentation Requirements

#### Code Documentation:
- README.md in every major directory
- API documentation using OpenAPI/Swagger
- Component prop documentation
- Setup instructions with prerequisites

#### User Documentation:
- Installation guide
- User manual for all features
- Troubleshooting section
- FAQ

### 8. Git Workflow

#### Commit Standards:
- Conventional commits format: `type(scope): description`
- Types: feat, fix, docs, style, refactor, test, chore
- Include ticket/issue number if applicable

#### Branch Strategy:
- `main` - production-ready code
- `develop` - integration branch
- `feature/*` - new features
- `fix/*` - bug fixes
- `docs/*` - documentation updates

### 9. Accessibility (A11y)
- WCAG 2.1 Level AA compliance
- Semantic HTML5 elements
- ARIA labels where needed
- Keyboard navigation support
- Screen reader compatibility
- Proper color contrast ratios

### 10. Content Standards

#### Writing Style:
- Clear, concise, and professional
- Active voice preferred
- Technical accuracy is paramount
- Include examples for complex concepts

#### Code Examples:
- Fully working, tested code
- Include comments explaining key concepts
- Follow project coding standards
- Provide both Python and JavaScript examples where applicable

## Non-Negotiables

1. **Mistral AI** must be used for all LLM functionality (NOT OpenAI)
2. **Spec-Kit Plus** must be used for development workflow
3. All features must work offline-first (with degraded functionality)
4. No plagiarism - all content must be original or properly cited
5. Mobile-first responsive design
6. Zero tolerance for security vulnerabilities
7. Full compliance with data privacy regulations (GDPR, etc.)

## Success Metrics

### Technical Metrics:
- All automated tests passing
- Lighthouse score > 90 (Performance, Accessibility, Best Practices, SEO)
- Zero console errors or warnings in production
- API uptime > 99.5%

### User Experience Metrics:
- RAG chatbot accuracy > 85%
- User can complete authentication flow < 2 minutes
- Content personalization reflects user background
- Translation maintains technical accuracy

### Project Metrics:
- All 17 chapters completed and published
- All 4 modules fully documented
- RAG chatbot answers questions from all chapters
- At least 3 bonus features implemented

## Governance

### Decision Making:
- Architectural decisions documented in ADRs
- Breaking changes require review
- Performance regressions not accepted
- Security issues have highest priority

### Review Process:
- All code changes must be tested
- Breaking changes documented
- Migrations tested with rollback plans

## Evolution

This constitution is a living document. Updates should be:
- Proposed with clear rationale
- Discussed with stakeholders
- Documented with version history
- Reflected in all relevant documentation

---

**Version:** 1.0  
**Last Updated:** 2025-12-14  
**Owner:** Project Team
