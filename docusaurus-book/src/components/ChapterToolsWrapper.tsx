import React, { useState, useEffect } from 'react';
import { useAuth } from '../context/AuthContext';
import ChapterTools from './ChapterTools';
import TextSelectionMenu from './TextSelectionMenu';
import ReadingProgress from './ReadingProgress';

interface ChapterToolsWrapperProps {
  chapterTitle: string;
  chapterPath: string;
}

export default function ChapterToolsWrapper({ chapterTitle, chapterPath }: ChapterToolsWrapperProps) {
  const { isAuthenticated, user } = useAuth();
  const [docContent, setDocContent] = useState<string>('');
  const [isContentLoaded, setIsContentLoaded] = useState(false);

  // Get original content from the page after it renders
  useEffect(() => {
    const timer = setTimeout(() => {
      const articleElement = document.querySelector('article.markdown') as HTMLElement | null;
      if (articleElement) {
        // Get text content for AI processing
        const content = articleElement.textContent || articleElement.innerText || '';
        setDocContent(content);
        setIsContentLoaded(true);
        console.log('[ChapterToolsWrapper] Content loaded, length:', content.length);
      } else {
        console.log('[ChapterToolsWrapper] Article element not found');
        setIsContentLoaded(true); // Still mark as loaded to show tools
      }
    }, 800); // Increased delay to ensure content is rendered

    return () => clearTimeout(timer);
  }, [chapterPath]);

  // Get user profile for personalization
  const userProfile = user ? {
    hardware_type: (user as any).hardware_type || 'laptop',
    has_gpu: (user as any).has_gpu || false,
    has_robot_kit: (user as any).has_robot_kit || false,
    robot_kit_type: (user as any).robot_kit_type,
    coding_experience: (user as any).coding_experience || 'intermediate',
    robotics_experience: (user as any).robotics_experience || 'beginner',
    ros_experience: (user as any).ros_experience || 'none',
    ml_experience: (user as any).ml_experience || 'beginner',
    learning_goals: (user as any).learning_goals || [],
    areas_of_interest: (user as any).areas_of_interest || [],
    preferred_pace: (user as any).preferred_pace || 'self_paced',
  } : undefined;

  const handleContentChange = (content: string, type: 'original' | 'personalized' | 'translated') => {
    // Handle personalized/translated content display
    console.log(`[ChapterToolsWrapper] Content view changed to: ${type}`);
  };

  // Debug logging
  console.log('[ChapterToolsWrapper] Render - isAuthenticated:', isAuthenticated, 'isContentLoaded:', isContentLoaded, 'docContent length:', docContent.length);

  return (
    <div style={{ position: 'relative', zIndex: 100 }}>
      {/* Reading Progress Bar - only for authenticated users */}
      {isAuthenticated && (
        <ReadingProgress 
          chapterPath={chapterPath} 
          chapterTitle={chapterTitle} 
        />
      )}
      
      {/* Chapter Tools (Personalize/Translate) - show for authenticated users */}
      {isAuthenticated && isContentLoaded && (
        <div style={{ margin: '1rem 0' }}>
          <ChapterTools
            chapterTitle={chapterTitle}
            chapterContent={docContent || 'Content loading...'}
            onContentChange={handleContentChange}
            userProfile={userProfile}
          />
        </div>
      )}
      
      {/* Loading indicator while waiting for content */}
      {isAuthenticated && !isContentLoaded && (
        <div style={{ 
          padding: '1rem', 
          background: 'linear-gradient(135deg, #f8f9ff 0%, #f0f4ff 100%)',
          borderRadius: '12px',
          margin: '1rem 0',
          textAlign: 'center',
          color: '#6366f1'
        }}>
          Loading chapter tools...
        </div>
      )}
      
      {/* Text Selection Menu - always available for text selection */}
      <TextSelectionMenu chapterContext={chapterTitle} />
    </div>
  );
}
