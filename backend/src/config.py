from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    # Application
    app_env: str = "development"
    debug: bool = True
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    cors_origins: str = "*"
    
    # OpenAI
    openai_api_key: str
    openai_model: str = "gpt-4-turbo-preview"
    openai_embedding_model: str = "text-embedding-3-small"
    
    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "book_content"
    
    # Database
    database_url: str
    
    # Authentication
    auth_secret: str
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


settings = Settings()


if __name__ == "__main__":
    print("✅ Configuration loaded successfully")
    print(f"Environment: {settings.app_env}")
    print(f"Debug: {settings.debug}")
    print(f"OpenAI Model: {settings.openai_model}")
    print(f"Qdrant Collection: {settings.qdrant_collection_name}")
