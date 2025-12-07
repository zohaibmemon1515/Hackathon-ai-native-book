# Tasks: UI & RAG Chatbot Upgrade for Physical AI Book

**Feature Branch**: `2-ui-rag-upgrade`
**Spec**: `specs/2-ui-rag-upgrade/spec.md`
**Plan**: `specs/2-ui-rag-upgrade/plan.md`
**Research**: `specs/2-ui-rag-upgrade/research.md`

---

## Format: `[ID] [P?] [Story?] Description with file path`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2)
-   Include exact file paths in descriptions

---

## Phase 1: Setup & Research Resolution (Foundational Infrastructure)

*Goal: Establish the basic project structure for the backend, provision necessary cloud resources, and resolve key design decisions.*

### Research Tasks (resolve `NEEDS CLARIFICATION` from `research.md`)

- [X] T001 Resolve UI/UX Design Direction (Question 1 in `specs/2-ui-rag-upgrade/research.md`)
    - **Acceptance Criteria**: Specific aesthetic direction (e.g., color palette, font styles, overall mood) for the premium visual redesign is chosen and documented.
    - **Affected Files & Folders**: `specs/2-ui-rag-upgrade/research.md`
    - **Dependencies**: User input for choice.
- [X] T002 Resolve RAG Embedding Model Version (Question 2 in `specs/2-ui-rag-upgrade/research.md`)
    - **Acceptance Criteria**: Specific OpenAI embedding model version is chosen and documented, balancing quality, cost, and compatibility.
    - **Affected Files & Folders**: `specs/2-ui-rag-upgrade/research.md`, `backend/config.py`
    - **Dependencies**: User input for choice.
- [X] T003 Resolve Chatbot User Interaction Logging Detail (Question 3 in `specs/2-ui-rag-upgrade/research.md`)
    - **Acceptance Criteria**: Level of detail for logging user interactions (minimal, moderate, or detailed) in Neon Serverless Postgres is chosen and documented.
    - **Affected Files & Folders**: `specs/2-ui-rag-upgrade/research.md`, `backend/models/schemas.py`, `backend/services/db_service.py`
    - **Dependencies**: User input for choice.

### Backend Project Setup Tasks

- [X] T004 Create new backend project directory `backend/`
- [X] T005 Create FastAPI application entry point `backend/main.py`
- [X] T006 Create `requirements.txt` for backend Python dependencies `backend/requirements.txt`
- [X] T007 Create `Dockerfile` for FastAPI application `backend/Dockerfile`
- [X] T008 Create `.env.example` for backend environment variables `backend/.env.example`
- [X] T009 Create `backend/api/` directory and `__init__.py` `backend/api/__init__.py`
- [X] T010 Create `backend/services/` directory and `__init__.py` `backend/services/__init__.py`
- [X] T011 Create `backend/models/` directory and `__init__.py` `backend/models/__init__.py`
- [X] T012 Create `backend/scripts/` directory `backend/scripts/`
- [X] T013 Create `config.py` for backend configurations `backend/config.py`

### Cloud Resource Provisioning Tasks

- [X] T014 Provision Neon Serverless Postgres instance
    - **Acceptance Criteria**: A Neon Serverless Postgres database instance is provisioned and its connection string is available.
    - **Affected Files & Folders**: External (Neon Console), `backend/.env.example`, `backend/config.py`
- [X] T015 Provision Qdrant Cloud Free Tier instance and create collection
    - **Acceptance Criteria**: A Qdrant Cloud instance is provisioned, an API key is obtained, and a collection (e.g., `book_embeddings`) is created.
    - **Affected Files & Folders**: External (Qdrant Cloud Console), `backend/.env.example`, `backend/config.py`

---

## Phase 2: UI/UX Premium Upgrade (US1)

*Goal: Implement a modern, visually appealing, and interactive user interface for the Docusaurus book without breaking existing content.*
*Independent Test: A user can navigate through the redesigned book interface and observe the updated typography, spacing, animations, and new hero section without any broken layouts or functionality of existing content.*

### Implementation Tasks

- [X] T016 [P] [US1] Update `book/src/css/custom.css` with premium visual redesign, typography, and spacing.
    - **Acceptance Criteria**: Global CSS styles reflect the chosen aesthetic, improving typography and spacing across the book.
    - **Affected Files & Folders**: `book/src/css/custom.css`
    - **Dependencies**: T001 (UI/UX Design Direction).
- [X] T017 [P] [US1] Add Framer Motion to Docusaurus `book/package.json` dependencies.
    - **Acceptance Criteria**: `framer-motion` is added to `book/package.json` and `npm install` runs successfully.
    - **Affected Files & Folders**: `book/package.json`
- [X] T018 [P] [US1] Create `HeroSection.jsx` custom React component `book/src/components/HeroSection.jsx`.
    - **Acceptance Criteria**: `HeroSection.jsx` is created, includes placeholder for humanoid robot/Physical AI theme, and is ready for content.
    - **Affected Files & Folders**: `book/src/components/HeroSection.jsx`
- [X] T019 [P] [US1] Integrate `HeroSection.jsx` into the Docusaurus layout (e.g., homepage, theme override).
    - **Acceptance Criteria**: The new `HeroSection` component is rendered on the homepage or relevant landing page.
    - **Affected Files & Folders**: `book/docusaurus.config.js`, `book/src/components/HeroSection.jsx`, possibly `book/src/theme/Layout/index.js` (if overriding).
- [X] T020 [P] [US1] Enhance code blocks, diagrams, and layout consistency through custom CSS.
    - **Acceptance Criteria**: Code blocks, diagrams, and overall layout elements show improved styling and consistency.
    - **Affected Files & Folders**: `book/src/css/custom.css`
    - **Dependencies**: T001 (UI/UX Design Direction).
- [X] T021 [P] [US1] Add AI-styled illustrations/icons to `book/static/images/`.
    - **Acceptance Criteria**: New visual assets (SVG, 3D-like illustrations) are added to the static image directory.
    - **Affected Files & Folders**: `book/static/images/`
    - **Dependencies**: External asset creation.
- [X] T022 [P] [US1] Implement a reusable `PremiumLayout` wrapper (if needed) for premium component styling.
    - **Acceptance Criteria**: A `PremiumLayout` component is created and applied to relevant pages, demonstrating enhanced styling.
    - **Affected Files & Folders**: `book/src/components/PremiumLayout.jsx`, Docusaurus theme overrides.
- [X] T023 [P] [US1] Ensure responsiveness and modern polish for all UI elements.
    - **Acceptance Criteria**: The Docusaurus site is fully responsive across common devices and screen sizes.
    - **Affected Files & Containers**: `book/src/css/custom.css`, various Docusaurus theme components.
- [X] T024 [P] [US1] Implement dark mode upgrade and animation polish using Framer Motion.
    - **Acceptance Criteria**: Dark mode is functional and visually appealing. Animations are smooth and consistent with Framer Motion style.
    - **Affected Files & Folders**: `book/src/css/custom.css`, `book/docusaurus.config.js`, custom React components.

---

## Phase 3: RAG Chatbot Backend (US2)

*Goal: Develop a robust, scalable, and secure FastAPI backend to power the RAG chatbot.*
*Independent Test: The FastAPI backend can accept API requests for chat, embedding, and logging, and interact correctly with OpenAI, Qdrant, and Neon Postgres.*

### Implementation Tasks

- [X] T025 [US2] Define Pydantic schemas for chat requests, responses, and log entries `backend/models/schemas.py`.
    - **Acceptance Criteria**: Pydantic models for `ChatRequest`, `ChatResponse`, `LogEntry` are defined.
    - **Affected Files & Folders**: `backend/models/schemas.py`
    - **Dependencies**: T003 (Chatbot User Interaction Logging Detail).
- [X] T026 [US2] Implement `db_service.py` for Neon Postgres interactions `backend/services/db_service.py`.
    - **Acceptance Criteria**: Functions for connecting to Neon, storing chat history, and logging user interactions are implemented.
    - **Affected Files & Folders**: `backend/services/db_service.py`
    - **Dependencies**: T014 (Provision Neon instance), T025.
- [X] T027 [US2] Implement `embedding_service.py` for OpenAI embedding model and Qdrant client `backend/services/embedding_service.py`.
    - **Acceptance Criteria**: Functions for generating embeddings using OpenAI and interacting with Qdrant for vector storage/retrieval are implemented.
    - **Affected Files & Folders**: `backend/services/embedding_service.py`
    - **Dependencies**: T002 (RAG Embedding Model Version), T015 (Provision Qdrant instance).
- [X] T028 [US2] Implement `chatbot_service.py` for RAG orchestration logic `backend/services/chatbot_service.py`.
    - **Acceptance Criteria**: Core RAG logic is implemented, including context retrieval from Qdrant and LLM interaction via OpenAI Agents/ChatKit.
    - **Affected Files & Folders**: `backend/services/chatbot_service.py`
    - **Dependencies**: T027.
- [X] T029 [US2] Implement API routes in `backend/api/routes.py` for `/chat`, `/embed`, `/health`.
    - **Acceptance Criteria**: FastAPI routes are defined, calling corresponding service functions.
    - **Affected Files & Folders**: `backend/api/routes.py`, `backend/main.py`
    - **Dependencies**: T026, T027, T028.
- [X] T030 [US2] Implement "select text + ask" mode logic in `chatbot_service.py`.
    - **Acceptance Criteria**: Chatbot can receive highlighted text as additional context and incorporate it into RAG.
    - **Affected Files & Folders**: `backend/services/chatbot_service.py`, `backend/models/schemas.py`
    - **Dependencies**: T028.
- [X] T031 [US2] Implement streaming responses for `/chat` endpoint.
    - **Acceptance Criteria**: Chatbot responses are streamed chunk by chunk to the frontend.
    - **Affected Files & Folders**: `backend/api/routes.py`, `backend/services/chatbot_service.py`
    - **Dependencies**: T029.
- [X] T032 [US2] Implement chat history memory system (`chat_sessions`, `messages` schemas) `backend/services/db_service.py`, `backend/models/schemas.py`.
    - **Acceptance Criteria**: Chatbot maintains short-term conversational context using Neon Postgres.
    - **Affected Files & Folders**: `backend/services/db_service.py`, `backend/models/schemas.py`
    - **Dependencies**: T026.
- [X] T033 [US2] Implement a redirect fallback when no relevant embeddings exist.
    - **Acceptance Criteria**: Chatbot gracefully handles cases where no relevant context is found, perhaps by stating it cannot answer based on book content.
    - **Affected Files & Folders**: `backend/services/chatbot_service.py`
    - **Dependencies**: T028.

---

## Phase 4: Embedding Pipeline (Content Ingestion)

*Goal: Develop and execute an offline pipeline to chunk, embed, and upload book content to Qdrant.*

### Implementation Tasks

- [X] T034 [P] Develop `embed_content.py` script for book content ingestion `backend/scripts/embed_content.py`.
    - **Acceptance Criteria**: Python script is capable of parsing Markdown files, chunking content, generating embeddings, and uploading to Qdrant.
    - **Affected Files & Folders**: `backend/scripts/embed_content.py`, `book/docs/**` (read-only).
    - **Dependencies**: T002 (Embedding Model Version), T015 (Provision Qdrant instance), T027.
- [X] T035 Execute `embed_content.py` to populate Qdrant with initial book embeddings.
    - **Acceptance Criteria**: Qdrant collection is populated with vector embeddings for all existing book content.
    - **Affected Files & Folders**: Qdrant Cloud.
    - **Dependencies**: T034.

---

## Phase 5: Chat UI Widget Integration into Docusaurus (US2)

*Goal: Integrate the RAG chatbot as a floating AI assistant within the Docusaurus UI.*
*Independent Test: The chatbot UI widget appears as a floating AI assistant on Docusaurus pages and can send/receive messages from the backend.*

### Implementation Tasks

- [X] T036 [P] [US2] Create `ChatbotWidget.jsx` React component `book/src/components/ChatbotWidget.jsx`.
    - **Acceptance Criteria**: React component for the floating chatbot UI is developed, including input field, message display, and "select text + ask" functionality.
    - **Affected Files & Folders**: `book/src/components/ChatbotWidget.jsx`
- [X] T037 [P] [US2] Integrate `ChatbotWidget.jsx` into the Docusaurus layout via theme overriding `book/src/theme/Layout/index.js`.
    - **Acceptance Criteria**: The `ChatbotWidget` appears as a floating AI assistant on all Docusaurus pages.
    - **Affected Files & Folders**: `book/src/theme/Layout/index.js` (or similar override), `book/docusaurus.config.js`
    - **Dependencies**: T036.
- [X] T038 [P] [US2] Implement frontend logic to send user queries and highlighted text to FastAPI backend.
    - **Acceptance Criteria**: User input from chat and selected text can be sent to the `/chat` endpoint.
    - **Affected Files & Folders**: `book/src/components/ChatbotWidget.jsx`
    - **Dependencies**: T029 (`/chat` endpoint).
- [X] T039 [P] [US2] Implement frontend logic to handle and display streaming responses from FastAPI backend.
    - **Acceptance Criteria**: Chatbot responses are displayed in real-time as they stream from the backend.
    - **Affected Files & Folders**: `book/src/components/ChatbotWidget.jsx`
    - **Dependencies**: T031 (streaming responses).

---

## Phase 6: Cross-Cutting Concerns & Polish

*Goal: Ensure security, deployment, testing, performance, and maintain overall quality and backward compatibility.*

### Security Tasks

- [X] T040 Secure FastAPI backend API endpoints with API key authentication `backend/api/routes.py`, `backend/config.py`.
    - **Acceptance Criteria**: Access to `/chat` and `/embed` endpoints requires a valid API key.
    - **Affected Files & Folders**: `backend/api/routes.py`, `backend/config.py`, `backend/main.py`
- [X] T041 Implement rate-limiting for FastAPI endpoints `backend/main.py`.
    - **Acceptance Criteria**: API endpoints are protected against excessive requests.
    - **Affected Files & Folders**: `backend/main.py`

### Deployment Tasks

- [X] T042 Create GitHub Actions workflow for Docusaurus deployment to GitHub Pages `.github/workflows/deploy-docusaurus.yml`.
    - **Acceptance Criteria**: Frontend automatically deploys to GitHub Pages on specified events.
    - **Affected Files & Folders**: `.github/workflows/deploy-docusaurus.yml`, `book/docusaurus.config.js`
- [X] T043 Create GitHub Actions workflow for FastAPI backend deployment to Render (or chosen provider) `.github/workflows/deploy-fastapi.yml`.
    - **Acceptance Criteria**: Backend automatically deploys to Render on specified events.
    - **Affected Files & Folders**: `.github/workflows/deploy-fastapi.yml`, `backend/Dockerfile`
- [X] T044 Configure environment variables on Render for FastAPI backend.
    - **Acceptance Criteria**: All necessary API keys and database connection strings are securely configured on the Render service.
    - **Affected Files & Folders**: Render console.

### Testing & Validation Tasks

- [X] T045 Write unit tests for FastAPI routes and service logic `backend/tests/unit/`.
    - **Acceptance Criteria**: Core backend functions and API routes are covered by unit tests, all tests pass.
    - **Affected Files & Folders**: `backend/tests/unit/`
- [X] T046 Write integration tests for FastAPI, Qdrant, and Neon interactions `backend/tests/integration/`.
    - **Acceptance Criteria**: Tests verify correct interaction with external services (mocked or test instances), all tests pass.
    - **Affected Files & Folders**: `backend/tests/integration/`
- [X] T047 Implement retrieval accuracy benchmarks for RAG pipeline `backend/tests/rag_benchmarks.py`.
    - **Acceptance Criteria**: Automated script to evaluate RAG retrieval performance against a test question set.
    - **Affected Files & Folders**: `backend/tests/rag_benchmarks.py`
- [X] T048 Perform end-to-end RAG evaluation, including hallucination checks.
    - **Acceptance Criteria**: Chatbot responses meet accuracy and relevance criteria; out-of-scope questions are handled gracefully.
    - **Affected Files & Folders**: `backend/tests/rag_evaluation.py` (or similar script), `specs/2-ui-rag-upgrade/spec.md` (SC-003, SC-004)
- [X] T049 Write UI integration tests for custom components and chatbot widget `book/tests/ui/`.
    - **Acceptance Criteria**: Tests ensure custom UI renders correctly and chatbot widget is functional.
    - **Affected Files & Folders**: `book/tests/ui/`
- [X] T050 Conduct manual user testing for UI/UX and chatbot experience.
    - **Acceptance Criteria**: User feedback collected, demonstrating positive user experience (SC-001).
    - **Affected Files & Folders**: User survey data.
- [X] T051 Verify backward compatibility with existing Docusaurus content and structure.
    - **Acceptance Criteria**: All existing book content renders and functions as before the upgrade (SC-007). No breaking changes introduced.
    - **Affected Files & Folders**: `book/docs/**`, `book/blog/**`

### Performance & Optimization Tasks

- [X] T052 Optimize Docusaurus site performance (e.g., image optimization, lazy loading, code splitting) `book/docusaurus.config.js`, `book/src/css/custom.css`.
    - **Acceptance Criteria**: Page load times meet NFR-001 (below 3 seconds).
    - **Affected Files & Folders**: `book/docusaurus.config.js`, `book/src/css/custom.css`, various custom components.
- [X] T053 Implement caching strategies for FastAPI backend (e.g., response caching, database query caching).
    - **Acceptance Criteria**: API response times meet performance goals.
    - **Affected Files & Folders**: `backend/main.py`, `backend/services/**`

### Maintenance & Logging Tasks

- [X] T054 Implement structured logging for FastAPI backend `backend/main.py`, `backend/config.py`.
    - **Acceptance Criteria**: Backend logs are structured and contain necessary information for debugging.
    - **Affected Files & Folders**: `backend/main.py`, `backend/config.py`, `backend/services/**`
- [X] T055 Set up basic monitoring and health checks for FastAPI service.
    - **Acceptance Criteria**: Uptime and performance of backend can be monitored (SC-005).
    - **Affected Files & Folders**: Render console.

---

## Dependencies (User Story Completion Order)

1.  **Phase 1: Setup & Research Resolution**: No dependencies. Must be completed first.
2.  **Phase 2: UI/UX Premium Upgrade (US1)**: Can proceed once Phase 1 is complete.
3.  **Phase 3: RAG Chatbot Backend (US2)**: Can proceed once Phase 1 is complete.
4.  **Phase 4: Embedding Pipeline**: Depends on Phase 3 completion (embedding service).
5.  **Phase 5: Chat UI Widget Integration (US2)**: Depends on Phase 3 (backend API) and Phase 4 (content embedded) completion.
6.  **Phase 6: Cross-Cutting Concerns & Polish**: Depends on completion of all prior phases.

### Within Each User Story/Phase

-   Research tasks should resolve before dependent implementation.
-   Database/resource provisioning before services using them.
-   Frontend components before their integration into layout.
-   Backend services before API routes using them.
-   Content ingestion before chat functionality relies on embeddings.

---

## Parallel Execution Examples

-   Many tasks within Phase 1 (Backend Project Setup) can be done in parallel.
-   UI enhancement tasks (T016-T024) can largely be parallelized once UI/UX direction (T001) is resolved.
-   FastAPI service implementation (T026-T028) can be parallelized, followed by API route creation (T029).
-   Deployment tasks (T042-T044) can be prepared in parallel once relevant codebases are stable.
-   Testing tasks (T045-T051) can be initiated as soon as the respective components are implemented.

---

## Implementation Strategy

### Incremental Delivery & Research-Concurrent Approach

The project will follow an incremental delivery approach, prioritizing User Story 1 (UI Upgrade) and User Story 2 (RAG Chatbot). Research will be conducted concurrently with planning and implementation, allowing for flexible decision-making as new information becomes available.

### Backend-First for Chatbot

For the RAG Chatbot, a backend-first approach will be used:
1.  Setup backend infrastructure and services.
2.  Develop and test the embedding and RAG logic independently.
3.  Then, integrate the frontend UI widget.

### Continuous Integration/Continuous Deployment (CI/CD)

Utilize GitHub Actions for automated testing and deployment of both the Docusaurus frontend and FastAPI backend, ensuring rapid feedback and consistent deployments.

---

## Notes

-   Strict adherence to backward compatibility is paramount.
-   All new code and styling must be additive, avoiding modification of existing Docusaurus core files unless absolutely necessary via theme overrides.
-   Utilize environment variables for all sensitive information (API keys, database credentials).
