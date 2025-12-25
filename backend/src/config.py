from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    # Application
    app_env: str = "development"
    debug: bool = True
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    cors_origins: str = "*"
    
    # OpenAI - Railway: Set these in environment variables
    openai_api_key: str = ""
    openai_model: str = "gpt-4-turbo-preview"
    openai_embedding_model: str = "text-embedding-3-small"

    # Qdrant - Railway: Set these in environment variables
    qdrant_url: str = ""
    qdrant_api_key: str = ""
    qdrant_collection_name: str = "book_content"

    # Database - Railway: Set this in environment variables
    database_url: str = ""

    # Authentication - Railway: Set this in environment variables
    auth_secret: str = "default-dev-secret-change-in-production"
    session_cookie_name: str = "session_token"
    session_max_age: int = 604800  # 7 days
    
    # Rate limiting
    rate_limit_requests_per_minute: int = 10
    
    class Config:
        env_file = ".env"
        case_sensitive = False
    
    @property
    def cors_origins_list(self) -> List[str]:
        # Handle wildcard CORS properly
        if self.cors_origins.strip() == "*":
            return ["*"]
        return [origin.strip() for origin in self.cors_origins.split(",")]

    def validate_production_config(self) -> List[str]:
        """Validate critical configuration for production deployment."""
        warnings = []

        if not self.openai_api_key:
            warnings.append("OPENAI_API_KEY is not set - AI features will not work")
        if not self.qdrant_url:
            warnings.append("QDRANT_URL is not set - vector search will not work")
        if not self.qdrant_api_key:
            warnings.append("QDRANT_API_KEY is not set - vector search will not work")
        if not self.database_url:
            warnings.append("DATABASE_URL is not set - using in-memory storage")
        if self.auth_secret == "default-dev-secret-change-in-production":
            warnings.append("AUTH_SECRET is using default value - change in production!")

        return warnings


settings = Settings()


if __name__ == "__main__":
    print("✅ Configuration loaded successfully")
    print(f"Environment: {settings.app_env}")
    print(f"Debug: {settings.debug}")
    print(f"OpenAI Model: {settings.openai_model}")
    print(f"Qdrant Collection: {settings.qdrant_collection_name}")
