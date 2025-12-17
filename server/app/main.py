from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from server.app.api import rag
from server.app.core import config
from contextlib import asynccontextmanager
from fastapi import FastAPI

app = FastAPI(
    title="Humanoid Robotics Textbook RAG API",
    description="An API for interacting with the RAG system for the textbook.",
    version="0.1.0",
)

# @app.on_event("startup")
# async def startup_event():
#     """A startup event that prints all registered routes."""
#     print("--- Registered Routes ---")
#     for route in app.routes:
#         if hasattr(route, "methods"):
#             print(f'{route.path} {route.methods}')
#     print("-------------------------")

@asynccontextmanager
async def lifespan(app: FastAPI):
    # --- Everything before 'yield' runs on STARTUP ---
    print("Server is starting up...")
    # If you have specific startup logic, put it here
    yield
    # --- Everything after 'yield' runs on SHUTDOWN ---
    print("Server is shutting down...")

app = FastAPI(lifespan=lifespan)
# Set up CORS
origins = [
    "http://localhost:3000", # The address of the Docusaurus dev server
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(rag.router, prefix="/rag", tags=["RAG"])

@app.get("/")
def read_root():
    return {"message": "Welcome to the RAG API"}
