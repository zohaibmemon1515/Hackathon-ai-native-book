import logging
import sys
from fastapi import FastAPI, Request, Response
from fastapi.responses import JSONResponse
from backend.api.routes import router as api_router
from backend.services.db_service import db_service
import time
from starlette.middleware.base import BaseHTTPMiddleware
from starlette.responses import Response as StarletteResponse
from starlette.types import ASGIApp

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(name)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger(__name__)

class CacheMiddleware(BaseHTTPMiddleware):
    def __init__(self, app: ASGIApp, cache_ttl: int = 60):
        super().__init__(app)
        self.cache = {}
        self.cache_ttl = cache_ttl

    async def dispatch(self, request: Request, call_next):
        if request.method == "GET" and request.url.path == "/health":
            if request.url.path in self.cache and (time.time() - self.cache[request.url.path]["timestamp"]) < self.cache_ttl:
                logger.info("Serving /health from cache")
                return self.cache[request.url.path]["response"]
            
            response = await call_next(request)
            if response.status_code == 200:
                response_body = b""
                async for chunk in response.body_iterator:
                    response_body += chunk
                cached_response = StarletteResponse(content=response_body, media_type=response.media_type)
                # Ensure headers are copied for the cached response
                for header_name, header_value in response.headers.items():
                    cached_response.headers[header_name] = header_value
                self.cache[request.url.path] = {"response": cached_response, "timestamp": time.time()}
                return cached_response
            return response
        return await call_next(request)


app = FastAPI()

# Placeholder for a simple rate limiting
RATE_LIMIT_DURATION = 60 # seconds
RATE_LIMIT_REQUESTS = 10 # requests per duration
# In a real app, use a more robust solution like `fastapi-limiter` or a distributed cache
request_timestamps = {}

@app.middleware("http")
async def rate_limit_middleware(request: Request, call_next):
    # Only apply to specific endpoints, e.g., /chat
    if request.url.path == "/chat":
        client_ip = request.client.host
        current_time = time.time()

        if client_ip not in request_timestamps:
            request_timestamps[client_ip] = []

        # Remove old timestamps
        request_timestamps[client_ip] = [
            ts for ts in request_timestamps[client_ip] if current_time - ts < RATE_LIMIT_DURATION
        ]

        if len(request_timestamps[client_ip]) >= RATE_LIMIT_REQUESTS:
            logger.warning(f"Rate limit exceeded for IP: {client_ip}")
            return JSONResponse(
                status_code=429,
                content={"detail": "Rate limit exceeded. Please try again later."},
                headers={"Retry-After": str(RATE_LIMIT_DURATION)}
            )
        
        request_timestamps[client_ip].append(current_time)
    
    response = await call_next(request)
    logger.info(f"Request: {request.method} {request.url.path} - Response: {response.status_code}")
    return response

app.add_middleware(CacheMiddleware, cache_ttl=60) # Cache /health endpoint for 60 seconds

@app.on_event("startup")
async def startup_event():
    logger.info("Starting up database connection and creating tables.")
    await db_service.connect()
    await db_service.create_tables()

@app.on_event("shutdown")
async def shutdown_event():
    logger.info("Shutting down database connection.")
    await db_service.disconnect()

app.include_router(api_router)

@app.get("/")
async def read_root():
    logger.info("Root endpoint accessed.")
    return {"message": "FastAPI Backend for Physical AI & Humanoid Robotics Book"}
