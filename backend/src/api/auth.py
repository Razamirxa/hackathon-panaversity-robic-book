"""
Authentication API routes with Personalization Onboarding
Provides email/password authentication with JWT sessions and user profiling
"""
from fastapi import APIRouter, HTTPException, Depends, Response, Request
from pydantic import BaseModel, EmailStr, Field
from sqlalchemy.orm import Session
from datetime import datetime, timedelta
import bcrypt
import jwt
import os
from typing import Optional, List

from ..database import get_db
from ..models.models import User

router = APIRouter(prefix="/auth", tags=["authentication"])

# JWT Configuration
JWT_SECRET = os.getenv("JWT_SECRET", os.getenv("OPENAI_API_KEY", "fallback-secret-key"))
JWT_ALGORITHM = "HS256"
JWT_EXPIRATION_DAYS = 7

# Request/Response models
class SignUpRequest(BaseModel):
    email: EmailStr
    password: str
    name: str

class SignInRequest(BaseModel):
    email: EmailStr
    password: str
    rememberMe: bool = True

# Onboarding Profile Models
class HardwareProfile(BaseModel):
    """Hardware background for personalized learning paths"""
    hardware_type: str = Field(..., description="laptop, raspberry_pi, jetson, cloud_only, other")
    hardware_specs: Optional[str] = Field(None, description="Additional hardware details")
    has_gpu: bool = False
    has_robot_kit: bool = False
    robot_kit_type: Optional[str] = None

class ExperienceProfile(BaseModel):
    """Software and experience background"""
    education_level: str = Field(..., description="high_school, undergraduate, graduate, professional")
    coding_experience: str = Field(..., description="beginner, intermediate, advanced, expert")
    robotics_experience: str = Field("none", description="none, beginner, intermediate, advanced")
    ml_experience: str = Field("none", description="none, beginner, intermediate, advanced")
    ros_experience: str = Field("none", description="none, beginner, intermediate, advanced")
    programming_languages: List[str] = Field(default_factory=list)

class GoalsProfile(BaseModel):
    """Learning goals and preferences"""
    learning_goals: List[str] = Field(default_factory=list)
    preferred_pace: str = Field("self_paced", description="self_paced, structured, intensive")
    time_commitment: str = Field("few_hours_week", description="few_hours_week, part_time, full_time")
    areas_of_interest: List[str] = Field(default_factory=list)

class OnboardingRequest(BaseModel):
    """Complete onboarding profile"""
    hardware: HardwareProfile
    experience: ExperienceProfile
    goals: GoalsProfile

class UserProfileResponse(BaseModel):
    """Full user profile with personalization data"""
    id: int
    email: str
    name: str
    onboarding_completed: bool
    created_at: datetime
    hardware_type: Optional[str] = None
    hardware_specs: Optional[str] = None
    has_gpu: bool = False
    has_robot_kit: bool = False
    robot_kit_type: Optional[str] = None
    education_level: Optional[str] = None
    coding_experience: Optional[str] = None
    robotics_experience: Optional[str] = None
    ml_experience: Optional[str] = None
    ros_experience: Optional[str] = None
    programming_languages: Optional[List[str]] = None
    learning_goals: Optional[List[str]] = None
    preferred_pace: Optional[str] = None
    time_commitment: Optional[str] = None
    areas_of_interest: Optional[List[str]] = None

    class Config:
        from_attributes = True

class UserResponse(BaseModel):
    id: int
    email: str
    name: str
    createdAt: datetime
    onboardingCompleted: bool = False

    class Config:
        from_attributes = True

class AuthResponse(BaseModel):
    user: UserResponse
    token: str
    needsOnboarding: bool = True

class SessionResponse(BaseModel):
    user: Optional[UserResponse] = None
    needsOnboarding: bool = False

# Helper functions
def hash_password(password: str) -> str:
    """Hash password using bcrypt"""
    return bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt()).decode('utf-8')

def verify_password(password: str, hashed: str) -> bool:
    """Verify password against hash"""
    return bcrypt.checkpw(password.encode('utf-8'), hashed.encode('utf-8'))

def create_token(user_id: int, email: str) -> str:
    """Create JWT token"""
    payload = {
        "sub": str(user_id),
        "email": email,
        "exp": datetime.utcnow() + timedelta(days=JWT_EXPIRATION_DAYS),
        "iat": datetime.utcnow()
    }
    return jwt.encode(payload, JWT_SECRET, algorithm=JWT_ALGORITHM)

def verify_token(token: str) -> dict:
    """Verify and decode JWT token"""
    try:
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")

def get_token_from_request(request: Request) -> Optional[str]:
    """Extract token from request (cookie or header)"""
    # Try cookie first
    token = request.cookies.get("hackathon_book_session")
    if token:
        return token
    
    # Try Authorization header
    auth_header = request.headers.get("Authorization")
    if auth_header and auth_header.startswith("Bearer "):
        return auth_header[7:]
    
    return None

# Routes
@router.post("/sign-up/email", response_model=AuthResponse)
async def sign_up(request: SignUpRequest, response: Response, db: Session = Depends(get_db)):
    """Sign up with email and password"""
    # Check if user exists
    existing_user = db.query(User).filter(User.email == request.email).first()
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    
    # Validate password
    if len(request.password) < 8:
        raise HTTPException(status_code=400, detail="Password must be at least 8 characters")
    if len(request.password) > 128:
        raise HTTPException(status_code=400, detail="Password must be at most 128 characters")
    
    # Create user
    hashed_password = hash_password(request.password)
    user = User(
        email=request.email,
        name=request.name,
        password_hash=hashed_password,
        onboarding_completed=False,
        created_at=datetime.utcnow()
    )
    db.add(user)
    db.commit()
    db.refresh(user)
    
    # Create token
    token = create_token(user.id, user.email)
    
    # Set cookie
    response.set_cookie(
        key="hackathon_book_session",
        value=token,
        httponly=True,
        secure=os.getenv("NODE_ENV") == "production",
        samesite="lax",
        max_age=60 * 60 * 24 * JWT_EXPIRATION_DAYS
    )
    
    return AuthResponse(
        user=UserResponse(
            id=user.id,
            email=user.email,
            name=user.name or "",
            createdAt=user.created_at,
            onboardingCompleted=False
        ),
        token=token,
        needsOnboarding=True
    )

@router.post("/sign-in/email", response_model=AuthResponse)
async def sign_in(request: SignInRequest, response: Response, db: Session = Depends(get_db)):
    """Sign in with email and password"""
    # Find user
    user = db.query(User).filter(User.email == request.email).first()
    if not user:
        raise HTTPException(status_code=401, detail="Invalid email or password")
    
    # Check if user has password (might be OAuth user)
    if not user.password_hash:
        raise HTTPException(status_code=401, detail="Invalid email or password")
    
    # Verify password
    if not verify_password(request.password, user.password_hash):
        raise HTTPException(status_code=401, detail="Invalid email or password")
    
    # Create token
    token = create_token(user.id, user.email)
    
    # Set cookie
    max_age = 60 * 60 * 24 * JWT_EXPIRATION_DAYS if request.rememberMe else None
    response.set_cookie(
        key="hackathon_book_session",
        value=token,
        httponly=True,
        secure=os.getenv("NODE_ENV") == "production",
        samesite="lax",
        max_age=max_age
    )
    
    return AuthResponse(
        user=UserResponse(
            id=user.id,
            email=user.email,
            name=user.name or "",
            createdAt=user.created_at,
            onboardingCompleted=user.onboarding_completed or False
        ),
        token=token,
        needsOnboarding=not (user.onboarding_completed or False)
    )

@router.post("/sign-out")
async def sign_out(response: Response):
    """Sign out and clear session"""
    response.delete_cookie(key="hackathon_book_session")
    return {"success": True}

@router.get("/session", response_model=SessionResponse)
async def get_session(request: Request, db: Session = Depends(get_db)):
    """Get current session/user"""
    token = get_token_from_request(request)
    
    if not token:
        return SessionResponse(user=None, needsOnboarding=False)
    
    try:
        payload = verify_token(token)
        user_id = int(payload["sub"])
        user = db.query(User).filter(User.id == user_id).first()
        
        if not user:
            return SessionResponse(user=None, needsOnboarding=False)
        
        return SessionResponse(
            user=UserResponse(
                id=user.id,
                email=user.email,
                name=user.name or "",
                createdAt=user.created_at,
                onboardingCompleted=user.onboarding_completed or False
            ),
            needsOnboarding=not (user.onboarding_completed or False)
        )
    except HTTPException:
        return SessionResponse(user=None, needsOnboarding=False)

@router.get("/get-session", response_model=SessionResponse)
async def get_session_alt(request: Request, db: Session = Depends(get_db)):
    """Alias for get session (Better Auth compatibility)"""
    return await get_session(request, db)


# Dependency to get current user
async def get_current_user(request: Request, db: Session = Depends(get_db)) -> Optional[User]:
    """Dependency to get the current authenticated user"""
    token = get_token_from_request(request)
    
    if not token:
        return None
    
    try:
        payload = verify_token(token)
        user_id = int(payload["sub"])
        return db.query(User).filter(User.id == user_id).first()
    except:
        return None

async def require_auth(request: Request, db: Session = Depends(get_db)) -> User:
    """Dependency that requires authentication"""
    user = await get_current_user(request, db)
    if not user:
        raise HTTPException(status_code=401, detail="Authentication required")
    return user

# ============ Onboarding Routes ============

@router.post("/onboarding", response_model=UserProfileResponse)
async def complete_onboarding(
    onboarding: OnboardingRequest,
    request: Request,
    db: Session = Depends(get_db)
):
    """Complete user onboarding with personalization profile"""
    token = get_token_from_request(request)
    if not token:
        raise HTTPException(status_code=401, detail="Authentication required")
    
    try:
        payload = verify_token(token)
        user_id = int(payload["sub"])
    except:
        raise HTTPException(status_code=401, detail="Invalid token")
    
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    
    # Update hardware profile
    user.hardware_type = onboarding.hardware.hardware_type
    user.hardware_specs = onboarding.hardware.hardware_specs
    user.has_gpu = onboarding.hardware.has_gpu
    user.has_robot_kit = onboarding.hardware.has_robot_kit
    user.robot_kit_type = onboarding.hardware.robot_kit_type
    
    # Update experience profile
    user.education_level = onboarding.experience.education_level
    user.coding_experience = onboarding.experience.coding_experience
    user.robotics_experience = onboarding.experience.robotics_experience
    user.ml_experience = onboarding.experience.ml_experience
    user.ros_experience = onboarding.experience.ros_experience
    user.programming_languages = onboarding.experience.programming_languages
    
    # Update goals profile
    user.learning_goals = onboarding.goals.learning_goals
    user.preferred_pace = onboarding.goals.preferred_pace
    user.time_commitment = onboarding.goals.time_commitment
    user.areas_of_interest = onboarding.goals.areas_of_interest
    
    # Mark onboarding as complete
    user.onboarding_completed = True
    
    db.commit()
    db.refresh(user)
    
    return UserProfileResponse(
        id=user.id,
        email=user.email,
        name=user.name or "",
        onboarding_completed=True,
        created_at=user.created_at,
        hardware_type=user.hardware_type,
        hardware_specs=user.hardware_specs,
        has_gpu=user.has_gpu or False,
        has_robot_kit=user.has_robot_kit or False,
        robot_kit_type=user.robot_kit_type,
        education_level=user.education_level,
        coding_experience=user.coding_experience,
        robotics_experience=user.robotics_experience,
        ml_experience=user.ml_experience,
        ros_experience=user.ros_experience,
        programming_languages=user.programming_languages,
        learning_goals=user.learning_goals,
        preferred_pace=user.preferred_pace,
        time_commitment=user.time_commitment,
        areas_of_interest=user.areas_of_interest
    )

@router.get("/profile", response_model=UserProfileResponse)
async def get_profile(request: Request, db: Session = Depends(get_db)):
    """Get current user's full profile"""
    token = get_token_from_request(request)
    if not token:
        raise HTTPException(status_code=401, detail="Authentication required")
    
    try:
        payload = verify_token(token)
        user_id = int(payload["sub"])
    except:
        raise HTTPException(status_code=401, detail="Invalid token")
    
    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(status_code=404, detail="User not found")
    
    return UserProfileResponse(
        id=user.id,
        email=user.email,
        name=user.name or "",
        onboarding_completed=user.onboarding_completed or False,
        created_at=user.created_at,
        hardware_type=user.hardware_type,
        hardware_specs=user.hardware_specs,
        has_gpu=user.has_gpu or False,
        has_robot_kit=user.has_robot_kit or False,
        robot_kit_type=user.robot_kit_type,
        education_level=user.education_level,
        coding_experience=user.coding_experience,
        robotics_experience=user.robotics_experience,
        ml_experience=user.ml_experience,
        ros_experience=user.ros_experience,
        programming_languages=user.programming_languages,
        learning_goals=user.learning_goals,
        preferred_pace=user.preferred_pace,
        time_commitment=user.time_commitment,
        areas_of_interest=user.areas_of_interest
    )

@router.patch("/profile", response_model=UserProfileResponse)
async def update_profile(
    profile_update: OnboardingRequest,
    request: Request,
    db: Session = Depends(get_db)
):
    """Update user profile"""
    return await complete_onboarding(profile_update, request, db)


def get_user_context(user: User) -> str:
    """Generate personalized context string for RAG based on user profile"""
    if not user or not user.onboarding_completed:
        return ""
    
    context_parts = []
    
    # Hardware context
    if user.hardware_type:
        hw_map = {
            "laptop": "using a laptop for development",
            "raspberry_pi": "working with Raspberry Pi hardware",
            "jetson": "using NVIDIA Jetson for edge AI",
            "cloud_only": "using cloud-based development",
        }
        context_parts.append(f"The user is {hw_map.get(user.hardware_type, user.hardware_type)}")
    
    if user.has_gpu:
        context_parts.append("and has GPU acceleration available")
    
    if user.has_robot_kit and user.robot_kit_type:
        context_parts.append(f"with a {user.robot_kit_type} robot kit")
    
    # Experience context
    exp_map = {"none": "no", "beginner": "beginner-level", "intermediate": "intermediate", "advanced": "advanced"}
    
    if user.coding_experience:
        context_parts.append(f"They have {user.coding_experience} coding experience")
    
    if user.robotics_experience and user.robotics_experience != "none":
        context_parts.append(f"and {exp_map.get(user.robotics_experience, user.robotics_experience)} robotics experience")
    
    if user.ml_experience and user.ml_experience != "none":
        context_parts.append(f"with {exp_map.get(user.ml_experience, user.ml_experience)} ML background")
    
    if user.ros_experience and user.ros_experience != "none":
        context_parts.append(f"and {exp_map.get(user.ros_experience, user.ros_experience)} ROS experience")
    
    # Education
    edu_map = {"high_school": "high school student", "undergraduate": "undergraduate student", 
               "graduate": "graduate student", "professional": "working professional"}
    if user.education_level:
        context_parts.append(f"They are a {edu_map.get(user.education_level, user.education_level)}")
    
    # Goals
    if user.learning_goals:
        goals_str = ", ".join(user.learning_goals)
        context_parts.append(f"Their learning goals include: {goals_str}")
    
    if user.areas_of_interest:
        interests_str = ", ".join(user.areas_of_interest)
        context_parts.append(f"They are interested in: {interests_str}")
    
    return ". ".join(context_parts) + "." if context_parts else ""
