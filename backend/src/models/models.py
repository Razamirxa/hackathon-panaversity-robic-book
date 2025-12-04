from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, Boolean, JSON, Float
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from ..database import Base

class User(Base):
    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    username = Column(String(255), unique=True, nullable=True, index=True)
    email = Column(String(255), unique=True, nullable=False, index=True)
    name = Column(String(255), nullable=True)
    password_hash = Column(String(255), nullable=True)
    email_verified = Column(Boolean, default=False)
    onboarding_completed = Column(Boolean, default=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
    
    # Personalization Profile Fields
    # Hardware Background
    hardware_type = Column(String(100), nullable=True)  # laptop, raspberry_pi, jetson, cloud_only, other
    hardware_specs = Column(Text, nullable=True)  # Additional hardware details
    has_gpu = Column(Boolean, default=False)
    has_robot_kit = Column(Boolean, default=False)
    robot_kit_type = Column(String(255), nullable=True)  # Type of robot kit if applicable
    
    # Software/Experience Background
    education_level = Column(String(100), nullable=True)  # high_school, undergraduate, graduate, professional
    coding_experience = Column(String(100), nullable=True)  # beginner, intermediate, advanced, expert
    robotics_experience = Column(String(100), nullable=True)  # none, beginner, intermediate, advanced
    ml_experience = Column(String(100), nullable=True)  # none, beginner, intermediate, advanced
    ros_experience = Column(String(100), nullable=True)  # none, beginner, intermediate, advanced
    programming_languages = Column(JSON, nullable=True)  # ["python", "cpp", "javascript"]
    
    # Learning Goals & Preferences
    learning_goals = Column(JSON, nullable=True)  # ["simulation", "physical_robots", "research", "career"]
    preferred_pace = Column(String(50), nullable=True)  # self_paced, structured, intensive
    time_commitment = Column(String(50), nullable=True)  # few_hours_week, part_time, full_time
    areas_of_interest = Column(JSON, nullable=True)  # ["humanoids", "manipulation", "navigation", "vision"]

    conversations = relationship("Conversation", back_populates="user", cascade="all, delete-orphan")
    reading_progress = relationship("ReadingProgress", back_populates="user", cascade="all, delete-orphan")


class ReadingProgress(Base):
    """Track user's reading progress for each chapter"""
    __tablename__ = "reading_progress"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    chapter_path = Column(String(500), nullable=False)  # e.g., "/physical-ai-textbook/ros2-fundamentals"
    chapter_title = Column(String(500), nullable=True)
    
    # Progress tracking
    is_completed = Column(Boolean, default=False)
    completion_percentage = Column(Float, default=0.0)  # 0-100
    scroll_position = Column(Float, default=0.0)  # Last scroll position (percentage)
    time_spent_seconds = Column(Integer, default=0)  # Total time spent reading
    
    # Timestamps
    first_visited = Column(DateTime(timezone=True), server_default=func.now())
    last_visited = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
    completed_at = Column(DateTime(timezone=True), nullable=True)
    
    user = relationship("User", back_populates="reading_progress")


class Conversation(Base):
    __tablename__ = "conversations"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    title = Column(String(500), nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())

    user = relationship("User", back_populates="conversations")
    messages = relationship("Message", back_populates="conversation", cascade="all, delete-orphan", order_by="Message.created_at")

class Message(Base):
    __tablename__ = "messages"

    id = Column(Integer, primary_key=True, index=True)
    conversation_id = Column(Integer, ForeignKey("conversations.id"), nullable=False)
    role = Column(String(50), nullable=False)  # 'user', 'assistant', 'system'
    content = Column(Text, nullable=False)
    selected_text = Column(Text, nullable=True)
    sources = Column(Text, nullable=True)  # JSON string of source references
    processing_time = Column(Integer, nullable=True)  # milliseconds
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    conversation = relationship("Conversation", back_populates="messages")
