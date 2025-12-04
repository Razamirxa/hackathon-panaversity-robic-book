"""
Content Service for Personalization, Translation, and Summarization
Handles AI-powered content transformation based on user profiles
"""
from typing import Optional, Dict, Any, List
from .openai_service import OpenAIService
import json


class ContentService:
    def __init__(self):
        self.openai_service = OpenAIService()

    def get_user_context(self, user_profile: Dict[str, Any]) -> str:
        """Build a context string from user profile for personalization"""
        context_parts = []
        
        # Hardware context
        hardware = user_profile.get('hardware_type', 'laptop')
        has_gpu = user_profile.get('has_gpu', False)
        has_robot = user_profile.get('has_robot_kit', False)
        robot_type = user_profile.get('robot_kit_type', '')
        
        if hardware == 'laptop':
            context_parts.append("User has a standard laptop without specialized hardware.")
            if has_gpu:
                context_parts.append("User has a GPU available for local processing.")
            else:
                context_parts.append("User should use cloud/Azure simulations instead of local GPU processing.")
        elif hardware == 'raspberry_pi':
            context_parts.append("User has a Raspberry Pi for edge deployment.")
        elif hardware == 'jetson':
            context_parts.append("User has NVIDIA Jetson for edge AI and robotics.")
        elif hardware == 'cloud_only':
            context_parts.append("User only has cloud access - all instructions should use cloud services.")
        
        if has_robot and robot_type:
            context_parts.append(f"User has a {robot_type} robot kit for physical experiments.")
        
        # Experience context
        coding_exp = user_profile.get('coding_experience', 'beginner')
        robotics_exp = user_profile.get('robotics_experience', 'none')
        ros_exp = user_profile.get('ros_experience', 'none')
        ml_exp = user_profile.get('ml_experience', 'none')
        
        if coding_exp in ['beginner', 'none']:
            context_parts.append("User is a beginner programmer - explain concepts thoroughly with analogies.")
        elif coding_exp == 'intermediate':
            context_parts.append("User has intermediate programming skills - balance explanation with code.")
        else:
            context_parts.append("User is an advanced programmer - be concise and technical.")
        
        if robotics_exp in ['none', 'beginner']:
            context_parts.append("User is new to robotics - explain robotics concepts from basics.")
        
        if ros_exp in ['none', 'beginner']:
            context_parts.append("User is new to ROS - explain ROS concepts carefully.")
        elif ros_exp in ['intermediate', 'advanced']:
            context_parts.append("User knows ROS - can skip basic ROS explanations.")
        
        # Goals context
        goals = user_profile.get('learning_goals', [])
        interests = user_profile.get('areas_of_interest', [])
        pace = user_profile.get('preferred_pace', 'self_paced')
        
        if goals:
            context_parts.append(f"User's learning goals: {', '.join(goals)}.")
        if interests:
            context_parts.append(f"User is especially interested in: {', '.join(interests)}.")
        
        return " ".join(context_parts)

    def personalize_content(
        self,
        content: str,
        user_profile: Dict[str, Any],
        chapter_title: str = ""
    ) -> Dict[str, Any]:
        """
        Personalize chapter content based on user profile
        
        Args:
            content: Original markdown content
            user_profile: User's profile data (hardware, experience, goals)
            chapter_title: Title of the current chapter
            
        Returns:
            Dict with personalized content and metadata
        """
        user_context = self.get_user_context(user_profile)
        
        # Determine experience level for tone
        coding_exp = user_profile.get('coding_experience', 'beginner')
        is_beginner = coding_exp in ['beginner', 'none']
        
        # Build personalization prompt
        system_prompt = f"""You are an expert technical writer adapting educational content about Physical AI and Robotics.

USER PROFILE:
{user_context}

YOUR TASK:
Rewrite the following content to be personalized for this specific user. Follow these rules:

1. HARDWARE ADAPTATION:
   - If user has standard laptop without GPU: Replace GPU-intensive instructions with Azure/cloud simulation alternatives
   - If user has Jetson/Raspberry Pi: Add physical deployment steps and edge-specific optimizations
   - If user has robot kit: Add hands-on exercises with their specific hardware

2. EXPERIENCE-BASED TONE:
   {"- Add 'Beginner Tips' boxes with analogies and step-by-step explanations" if is_beginner else "- Be concise and technical, skip basic explanations"}
   {"- Use simple analogies (e.g., 'Think of ROS nodes like apps on your phone')" if is_beginner else "- Include advanced code patterns and optimizations"}
   {"- Add encouragement and 'take your time' notes" if is_beginner else "- Focus on efficiency and best practices"}

3. FORMAT:
   - Keep the markdown structure (headers, code blocks, lists)
   - Add [PERSONALIZED] markers for adapted sections
   - Preserve all original code examples but adapt comments/explanations

Return ONLY the personalized markdown content."""

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Chapter: {chapter_title}\n\nContent to personalize:\n\n{content}"}
        ]

        try:
            personalized = self.openai_service.chat_completion(
                messages=messages,
                temperature=0.7,
                max_tokens=4000
            )
            
            return {
                "success": True,
                "personalized_content": personalized,
                "adaptations": {
                    "hardware_type": user_profile.get('hardware_type', 'laptop'),
                    "experience_level": coding_exp,
                    "tone": "beginner-friendly" if is_beginner else "technical"
                }
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "personalized_content": content  # Return original on error
            }

    def translate_content(
        self,
        content: str,
        target_language: str = "Urdu",
        preserve_code: bool = True
    ) -> Dict[str, Any]:
        """
        Translate content to target language (default Urdu)
        
        Args:
            content: Original content to translate
            target_language: Target language (default: Urdu)
            preserve_code: Whether to keep code blocks in English
            
        Returns:
            Dict with translated content
        """
        system_prompt = f"""You are an expert translator specializing in technical educational content about AI and Robotics.

TRANSLATE the following content to {target_language}.

RULES:
1. Translate all explanatory text to {target_language}
2. {"Keep all code blocks, variable names, and technical commands in English" if preserve_code else "Translate everything including comments"}
3. Keep technical terms in English with {target_language} explanation in parentheses where helpful
4. Maintain markdown formatting (headers, lists, code blocks)
5. Preserve the educational tone and context
6. For Urdu: Use proper Urdu script (نستعلیق), right-to-left text where appropriate

Return ONLY the translated content in markdown format."""

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": content}
        ]

        try:
            translated = self.openai_service.chat_completion(
                messages=messages,
                temperature=0.3,  # Lower temperature for more accurate translation
                max_tokens=4000
            )
            
            return {
                "success": True,
                "translated_content": translated,
                "target_language": target_language
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "translated_content": content
            }

    def summarize_content(
        self,
        content: str,
        summary_type: str = "brief",  # brief, detailed, bullet_points
        user_level: str = "intermediate"
    ) -> Dict[str, Any]:
        """
        Generate a summary of the content
        
        Args:
            content: Content to summarize
            summary_type: Type of summary (brief, detailed, bullet_points)
            user_level: User's experience level for appropriate complexity
            
        Returns:
            Dict with summary
        """
        type_instructions = {
            "brief": "Create a 2-3 sentence summary capturing the key points.",
            "detailed": "Create a comprehensive summary covering all main concepts and their relationships.",
            "bullet_points": "Create a bullet-point summary with key takeaways and action items."
        }
        
        level_tone = {
            "beginner": "Use simple language and analogies.",
            "intermediate": "Use clear technical language.",
            "advanced": "Be concise and technical."
        }
        
        system_prompt = f"""You are an expert at summarizing technical educational content about Physical AI and Robotics.

{type_instructions.get(summary_type, type_instructions['brief'])}
{level_tone.get(user_level, level_tone['intermediate'])}

Focus on:
- Key concepts and their practical applications
- Important code patterns or commands mentioned
- Prerequisites and dependencies
- Action items for the learner"""

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": f"Summarize this content:\n\n{content}"}
        ]

        try:
            summary = self.openai_service.chat_completion(
                messages=messages,
                temperature=0.5,
                max_tokens=1000
            )
            
            return {
                "success": True,
                "summary": summary,
                "summary_type": summary_type
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "summary": ""
            }

    def explain_selection(
        self,
        selected_text: str,
        chapter_context: str = "",
        user_profile: Optional[Dict[str, Any]] = None,
        question: str = ""
    ) -> Dict[str, Any]:
        """
        Explain selected text in context
        
        Args:
            selected_text: The text user selected
            chapter_context: Context about current chapter
            user_profile: Optional user profile for personalization
            question: Optional specific question about the selection
            
        Returns:
            Dict with explanation
        """
        # Determine user level
        user_level = "intermediate"
        if user_profile:
            coding_exp = user_profile.get('coding_experience', 'intermediate')
            if coding_exp in ['beginner', 'none']:
                user_level = "beginner"
            elif coding_exp in ['advanced', 'expert']:
                user_level = "advanced"
        
        level_instructions = {
            "beginner": "Explain like I'm new to programming. Use simple analogies and examples.",
            "intermediate": "Provide a clear technical explanation with relevant examples.",
            "advanced": "Be concise and technical. Focus on nuances and advanced implications."
        }
        
        system_prompt = f"""You are an expert tutor helping a student understand Physical AI and Robotics concepts.

{level_instructions.get(user_level, level_instructions['intermediate'])}

{"Chapter context: " + chapter_context if chapter_context else ""}

The student has selected the following text and wants to understand it better:"""

        user_message = f'Selected text: "{selected_text}"'
        if question:
            user_message += f"\n\nStudent's question: {question}"
        else:
            user_message += "\n\nPlease explain this clearly."

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]

        try:
            explanation = self.openai_service.chat_completion(
                messages=messages,
                temperature=0.7,
                max_tokens=1500
            )
            
            return {
                "success": True,
                "explanation": explanation,
                "user_level": user_level
            }
        except Exception as e:
            return {
                "success": False,
                "error": str(e),
                "explanation": ""
            }
