"""
Progress Tracking API Routes
Track user reading progress across chapters
"""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime
from ..database import get_db
from ..models.models import User, ReadingProgress
from sqlalchemy.orm import Session
from sqlalchemy import func

router = APIRouter()


class UpdateProgressRequest(BaseModel):
    user_id: int = Field(..., description="User ID")
    chapter_path: str = Field(..., description="Chapter path (e.g., /physical-ai-textbook/ros2-fundamentals)")
    chapter_title: str = Field("", description="Chapter title")
    scroll_position: float = Field(0.0, description="Scroll position percentage (0-100)")
    time_spent_seconds: int = Field(0, description="Time spent reading in seconds")
    is_completed: bool = Field(False, description="Whether chapter is completed")


class ProgressResponse(BaseModel):
    id: int
    chapter_path: str
    chapter_title: Optional[str]
    is_completed: bool
    completion_percentage: float
    scroll_position: float
    time_spent_seconds: int
    first_visited: datetime
    last_visited: datetime
    completed_at: Optional[datetime]


class UserProgressSummary(BaseModel):
    total_chapters_visited: int
    total_chapters_completed: int
    total_time_spent_seconds: int
    completion_percentage: float
    chapters: List[ProgressResponse]


@router.post("/update", response_model=ProgressResponse)
async def update_progress(request: UpdateProgressRequest, db: Session = Depends(get_db)):
    """
    Update reading progress for a chapter
    
    - Creates new progress record if first visit
    - Updates scroll position, time spent, and completion status
    """
    try:
        # Check if progress record exists
        progress = db.query(ReadingProgress).filter(
            ReadingProgress.user_id == request.user_id,
            ReadingProgress.chapter_path == request.chapter_path
        ).first()
        
        if progress:
            # Update existing progress
            progress.scroll_position = max(progress.scroll_position, request.scroll_position)
            progress.time_spent_seconds += request.time_spent_seconds
            progress.completion_percentage = request.scroll_position
            
            if request.is_completed and not progress.is_completed:
                progress.is_completed = True
                progress.completion_percentage = 100.0
                progress.completed_at = datetime.utcnow()
            
            if request.chapter_title:
                progress.chapter_title = request.chapter_title
        else:
            # Create new progress record
            progress = ReadingProgress(
                user_id=request.user_id,
                chapter_path=request.chapter_path,
                chapter_title=request.chapter_title,
                scroll_position=request.scroll_position,
                completion_percentage=request.scroll_position,
                time_spent_seconds=request.time_spent_seconds,
                is_completed=request.is_completed
            )
            
            if request.is_completed:
                progress.completed_at = datetime.utcnow()
                progress.completion_percentage = 100.0
            
            db.add(progress)
        
        db.commit()
        db.refresh(progress)
        
        return ProgressResponse(
            id=progress.id,
            chapter_path=progress.chapter_path,
            chapter_title=progress.chapter_title,
            is_completed=progress.is_completed,
            completion_percentage=progress.completion_percentage,
            scroll_position=progress.scroll_position,
            time_spent_seconds=progress.time_spent_seconds,
            first_visited=progress.first_visited,
            last_visited=progress.last_visited,
            completed_at=progress.completed_at
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/user/{user_id}", response_model=UserProgressSummary)
async def get_user_progress(user_id: int, db: Session = Depends(get_db)):
    """
    Get all reading progress for a user
    
    - Returns summary statistics
    - Lists all chapters with progress data
    """
    try:
        # Get all progress records for user
        progress_records = db.query(ReadingProgress).filter(
            ReadingProgress.user_id == user_id
        ).order_by(ReadingProgress.last_visited.desc()).all()
        
        # Calculate summary statistics
        total_visited = len(progress_records)
        total_completed = sum(1 for p in progress_records if p.is_completed)
        total_time = sum(p.time_spent_seconds for p in progress_records)
        
        # Assume 50 chapters total (adjust based on actual book)
        total_chapters = 50
        overall_completion = (total_completed / total_chapters) * 100 if total_chapters > 0 else 0
        
        chapters = [
            ProgressResponse(
                id=p.id,
                chapter_path=p.chapter_path,
                chapter_title=p.chapter_title,
                is_completed=p.is_completed,
                completion_percentage=p.completion_percentage,
                scroll_position=p.scroll_position,
                time_spent_seconds=p.time_spent_seconds,
                first_visited=p.first_visited,
                last_visited=p.last_visited,
                completed_at=p.completed_at
            )
            for p in progress_records
        ]
        
        return UserProgressSummary(
            total_chapters_visited=total_visited,
            total_chapters_completed=total_completed,
            total_time_spent_seconds=total_time,
            completion_percentage=overall_completion,
            chapters=chapters
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/chapter/{user_id}", response_model=Optional[ProgressResponse])
async def get_chapter_progress(
    user_id: int,
    chapter_path: str,
    db: Session = Depends(get_db)
):
    """
    Get progress for a specific chapter
    """
    try:
        progress = db.query(ReadingProgress).filter(
            ReadingProgress.user_id == user_id,
            ReadingProgress.chapter_path == chapter_path
        ).first()
        
        if not progress:
            return None
        
        return ProgressResponse(
            id=progress.id,
            chapter_path=progress.chapter_path,
            chapter_title=progress.chapter_title,
            is_completed=progress.is_completed,
            completion_percentage=progress.completion_percentage,
            scroll_position=progress.scroll_position,
            time_spent_seconds=progress.time_spent_seconds,
            first_visited=progress.first_visited,
            last_visited=progress.last_visited,
            completed_at=progress.completed_at
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/mark-complete/{user_id}")
async def mark_chapter_complete(
    user_id: int,
    chapter_path: str,
    chapter_title: str = "",
    db: Session = Depends(get_db)
):
    """
    Mark a chapter as completed
    """
    try:
        progress = db.query(ReadingProgress).filter(
            ReadingProgress.user_id == user_id,
            ReadingProgress.chapter_path == chapter_path
        ).first()
        
        if progress:
            progress.is_completed = True
            progress.completion_percentage = 100.0
            progress.completed_at = datetime.utcnow()
        else:
            progress = ReadingProgress(
                user_id=user_id,
                chapter_path=chapter_path,
                chapter_title=chapter_title,
                is_completed=True,
                completion_percentage=100.0,
                completed_at=datetime.utcnow()
            )
            db.add(progress)
        
        db.commit()
        
        return {"success": True, "message": "Chapter marked as complete"}
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
