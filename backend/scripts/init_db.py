import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from sqlalchemy import create_engine, text
from dotenv import load_dotenv

load_dotenv()

DATABASE_URL = os.getenv("DATABASE_URL")

if not DATABASE_URL:
    print("❌ DATABASE_URL not found in environment variables")
    sys.exit(1)

engine = create_engine(DATABASE_URL)

schema_sql = """
-- Users table
CREATE TABLE IF NOT EXISTS users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  software_background VARCHAR(50) NOT NULL CHECK (software_background IN ('beginner', 'intermediate', 'advanced')),
  hardware_background VARCHAR(50) NOT NULL CHECK (hardware_background IN ('simulation-only', 'hobbyist', 'professional')),
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

-- Conversations table
CREATE TABLE IF NOT EXISTS conversations (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  title VARCHAR(255),
  language VARCHAR(2) DEFAULT 'en' CHECK (language IN ('en', 'ur')),
  created_at TIMESTAMP DEFAULT NOW(),
  updated_at TIMESTAMP DEFAULT NOW()
);

-- Messages table
CREATE TABLE IF NOT EXISTS messages (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
  role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
  content TEXT NOT NULL,
  citations JSONB,
  created_at TIMESTAMP DEFAULT NOW()
);

-- Personalized content cache
CREATE TABLE IF NOT EXISTS personalized_content (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
  chapter_id VARCHAR(255) NOT NULL,
  software_background VARCHAR(50),
  hardware_background VARCHAR(50),
  personalized_content TEXT NOT NULL,
  created_at TIMESTAMP DEFAULT NOW(),
  expires_at TIMESTAMP NOT NULL,
  UNIQUE(user_id, chapter_id)
);

-- Translated content cache
CREATE TABLE IF NOT EXISTS translated_content (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  chapter_id VARCHAR(255) NOT NULL UNIQUE,
  language VARCHAR(2) DEFAULT 'ur',
  translated_content TEXT NOT NULL,
  code_blocks_preserved BOOLEAN DEFAULT TRUE,
  created_at TIMESTAMP DEFAULT NOW(),
  expires_at TIMESTAMP NOT NULL
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_conversations_user ON conversations(user_id, updated_at DESC);
CREATE INDEX IF NOT EXISTS idx_messages_conversation ON messages(conversation_id, created_at);
CREATE INDEX IF NOT EXISTS idx_personalized_user_chapter ON personalized_content(user_id, chapter_id);
CREATE INDEX IF NOT EXISTS idx_translated_chapter ON translated_content(chapter_id);
"""

try:
    with engine.connect() as conn:
        conn.execute(text(schema_sql))
        conn.commit()
        print("✅ Database schema initialized successfully")
        print("Tables created: users, conversations, messages, personalized_content, translated_content")
except Exception as e:
    print(f"❌ Error initializing database: {e}")
    sys.exit(1)
