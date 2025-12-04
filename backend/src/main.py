from dotenv import load_dotenv

load_dotenv()  # Load environment variables from .env file

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from .api.chat import router as chat_router
from .api.auth import router as auth_router
from .api.content import router as content_router
from .api.progress import router as progress_router

app = FastAPI(
    title="Book-Embedded RAG Chatbot API",
    description="RAG chatbot for 'Physical AI & Humanoid Robotics' book",
    version="1.0.0"
)

# CORS configuration - allow frontend origins
allowed_origins = [
    "http://localhost:3000",
    "http://localhost:3001",
    "https://razamirxa.github.io",  # GitHub Pages
    "https://Razamirxa.github.io",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers
app.include_router(chat_router, prefix="/api", tags=["chat"])
app.include_router(auth_router, prefix="/api", tags=["authentication"])
app.include_router(content_router, prefix="/api/content", tags=["content"])
app.include_router(progress_router, prefix="/api/progress", tags=["progress"])

@app.get("/")
async def read_root():
    return {
        "message": "Book-Embedded RAG Chatbot API",
        "status": "running",
        "docs": "/docs"
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
