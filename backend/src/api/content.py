"""
Content API Routes for Personalization, Translation, and Summarization
"""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List
from ..services.content_service import ContentService
from ..database import get_db
from ..models.models import User
from sqlalchemy.orm import Session

router = APIRouter()

# Initialize Content Service
content_service = ContentService()


class PersonalizeRequest(BaseModel):
    content: str = Field(..., description="Original markdown content to personalize")
    chapter_title: str = Field("", description="Title of the current chapter")
    user_id: Optional[int] = Field(None, description="User ID for profile lookup")
    user_profile: Optional[Dict[str, Any]] = Field(None, description="Direct user profile data")


class PersonalizeResponse(BaseModel):
    success: bool
    personalized_content: str
    adaptations: Optional[Dict[str, Any]] = None
    error: Optional[str] = None


class TranslateRequest(BaseModel):
    content: str = Field(..., description="Content to translate")
    target_language: str = Field("Urdu", description="Target language for translation")
    preserve_code: bool = Field(True, description="Keep code blocks in English")


class TranslateResponse(BaseModel):
    success: bool
    translated_content: str
    target_language: str
    error: Optional[str] = None


class SummarizeRequest(BaseModel):
    content: str = Field(..., description="Content to summarize")
    summary_type: str = Field("brief", description="Type: brief, detailed, bullet_points")
    user_level: str = Field("intermediate", description="User experience level")


class SummarizeResponse(BaseModel):
    success: bool
    summary: str
    summary_type: str
    error: Optional[str] = None


class ExplainRequest(BaseModel):
    selected_text: str = Field(..., description="Text selected by user")
    chapter_context: str = Field("", description="Context about current chapter")
    question: str = Field("", description="Specific question about the selection")
    user_id: Optional[int] = Field(None, description="User ID for profile lookup")


class ExplainResponse(BaseModel):
    success: bool
    explanation: str
    user_level: str
    error: Optional[str] = None


@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest, db: Session = Depends(get_db)):
    """
    Personalize chapter content based on user profile
    
    - Adapts content based on hardware (laptop, Jetson, cloud)
    - Adjusts tone for experience level (beginner/advanced)
    - Adds relevant tips and examples
    """
    try:
        # Get user profile
        user_profile = request.user_profile
        
        if not user_profile and request.user_id:
            user = db.query(User).filter(User.id == request.user_id).first()
            if user:
                user_profile = {
                    "hardware_type": user.hardware_type,
                    "hardware_specs": user.hardware_specs,
                    "has_gpu": user.has_gpu,
                    "has_robot_kit": user.has_robot_kit,
                    "robot_kit_type": user.robot_kit_type,
                    "coding_experience": user.coding_experience,
                    "robotics_experience": user.robotics_experience,
                    "ros_experience": user.ros_experience,
                    "ml_experience": user.ml_experience,
                    "learning_goals": user.learning_goals,
                    "areas_of_interest": user.areas_of_interest,
                    "preferred_pace": user.preferred_pace
                }
        
        if not user_profile:
            # Default profile for anonymous users
            user_profile = {
                "hardware_type": "laptop",
                "has_gpu": False,
                "has_robot_kit": False,
                "coding_experience": "intermediate"
            }
        
        result = content_service.personalize_content(
            content=request.content,
            user_profile=user_profile,
            chapter_title=request.chapter_title
        )
        
        return PersonalizeResponse(**result)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate content to target language (default: Urdu)
    
    - Preserves code blocks in English by default
    - Maintains markdown formatting
    - Keeps technical terms with translations in parentheses
    """
    try:
        result = content_service.translate_content(
            content=request.content,
            target_language=request.target_language,
            preserve_code=request.preserve_code
        )
        
        return TranslateResponse(**result)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/summarize", response_model=SummarizeResponse)
async def summarize_content(request: SummarizeRequest):
    """
    Generate a summary of content
    
    - Types: brief (2-3 sentences), detailed, bullet_points
    - Adapts complexity based on user level
    """
    try:
        result = content_service.summarize_content(
            content=request.content,
            summary_type=request.summary_type,
            user_level=request.user_level
        )
        
        return SummarizeResponse(**result)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/explain", response_model=ExplainResponse)
async def explain_selection(request: ExplainRequest, db: Session = Depends(get_db)):
    """
    Explain selected text in context
    
    - Personalized explanation based on user level
    - Can include specific questions about the selection
    """
    try:
        # Get user profile if user_id provided
        user_profile = None
        if request.user_id:
            user = db.query(User).filter(User.id == request.user_id).first()
            if user:
                user_profile = {
                    "coding_experience": user.coding_experience,
                    "robotics_experience": user.robotics_experience,
                    "ros_experience": user.ros_experience
                }
        
        result = content_service.explain_selection(
            selected_text=request.selected_text,
            chapter_context=request.chapter_context,
            user_profile=user_profile,
            question=request.question
        )
        
        return ExplainResponse(**result)
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
