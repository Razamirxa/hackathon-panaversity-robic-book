import React, { useState } from 'react';
import { useAuth } from '../context/AuthContext';
import { contentAPI, UserProfile } from '../services/content_api';
import styles from './ChapterTools.module.css';

interface ChapterToolsProps {
  chapterTitle: string;
  chapterContent: string;
  onContentChange: (content: string, type: 'original' | 'personalized' | 'translated') => void;
  userProfile?: UserProfile;
}

export default function ChapterTools({
  chapterTitle,
  chapterContent,
  onContentChange,
  userProfile,
}: ChapterToolsProps) {
  const { user, isAuthenticated } = useAuth();
  const [activeView, setActiveView] = useState<'original' | 'personalized' | 'translated'>('original');
  const [isPersonalizing, setIsPersonalizing] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);
  const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);

  const handlePersonalize = async () => {
    if (!isAuthenticated) {
      setError('Please sign in to personalize content');
      return;
    }

    setIsPersonalizing(true);
    setError(null);

    try {
      const response = await contentAPI.personalize({
        content: chapterContent,
        chapter_title: chapterTitle,
        user_id: user?.id,
        user_profile: userProfile,
      });

      if (response.success) {
        setPersonalizedContent(response.personalized_content);
        setActiveView('personalized');
        onContentChange(response.personalized_content, 'personalized');
      } else {
        setError(response.error || 'Personalization failed');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Personalization failed');
    } finally {
      setIsPersonalizing(false);
    }
  };

  const handleTranslate = async () => {
    setIsTranslating(true);
    setError(null);

    try {
      const contentToTranslate = activeView === 'personalized' && personalizedContent 
        ? personalizedContent 
        : chapterContent;

      const response = await contentAPI.translate({
        content: contentToTranslate,
        target_language: 'Urdu',
        preserve_code: true,
      });

      if (response.success) {
        setTranslatedContent(response.translated_content);
        setActiveView('translated');
        onContentChange(response.translated_content, 'translated');
      } else {
        setError(response.error || 'Translation failed');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Translation failed');
    } finally {
      setIsTranslating(false);
    }
  };

  const handleViewChange = (view: 'original' | 'personalized' | 'translated') => {
    setActiveView(view);
    
    if (view === 'original') {
      onContentChange(chapterContent, 'original');
    } else if (view === 'personalized' && personalizedContent) {
      onContentChange(personalizedContent, 'personalized');
    } else if (view === 'translated' && translatedContent) {
      onContentChange(translatedContent, 'translated');
    }
  };

  return (
    <div className={styles.chapterTools}>
      <div className={styles.toolsHeader}>
        <h4 className={styles.toolsTitle}>📚 Chapter Tools</h4>
        <p className={styles.toolsSubtitle}>Personalize your learning experience</p>
      </div>

      <div className={styles.toolsActions}>
        <button
          className={`${styles.toolButton} ${styles.personalizeButton}`}
          onClick={handlePersonalize}
          disabled={isPersonalizing || !isAuthenticated}
          title={!isAuthenticated ? 'Sign in to personalize' : 'Personalize content for your profile'}
        >
          {isPersonalizing ? (
            <>
              <span className={styles.spinner}></span>
              Personalizing...
            </>
          ) : (
            <>
              <span className={styles.icon}>🎯</span>
              Personalize for Me
            </>
          )}
        </button>

        <button
          className={`${styles.toolButton} ${styles.translateButton}`}
          onClick={handleTranslate}
          disabled={isTranslating}
        >
          {isTranslating ? (
            <>
              <span className={styles.spinner}></span>
              Translating...
            </>
          ) : (
            <>
              <span className={styles.icon}>🌐</span>
              Translate to Urdu
            </>
          )}
        </button>
      </div>

      {/* View Toggle Tabs */}
      {(personalizedContent || translatedContent) && (
        <div className={styles.viewTabs}>
          <button
            className={`${styles.tabButton} ${activeView === 'original' ? styles.activeTab : ''}`}
            onClick={() => handleViewChange('original')}
          >
            📄 Original
          </button>
          {personalizedContent && (
            <button
              className={`${styles.tabButton} ${activeView === 'personalized' ? styles.activeTab : ''}`}
              onClick={() => handleViewChange('personalized')}
            >
              🎯 Personalized
            </button>
          )}
          {translatedContent && (
            <button
              className={`${styles.tabButton} ${activeView === 'translated' ? styles.activeTab : ''}`}
              onClick={() => handleViewChange('translated')}
            >
              🌐 Urdu
            </button>
          )}
        </div>
      )}

      {/* Status Indicator */}
      {activeView !== 'original' && (
        <div className={styles.statusBadge}>
          {activeView === 'personalized' && (
            <span className={styles.personalizedBadge}>
              ✨ Content personalized for your {userProfile?.coding_experience || 'experience'} level
            </span>
          )}
          {activeView === 'translated' && (
            <span className={styles.translatedBadge}>
              🌐 اردو میں ترجمہ شدہ (Translated to Urdu)
            </span>
          )}
        </div>
      )}

      {error && (
        <div className={styles.errorMessage}>
          ⚠️ {error}
        </div>
      )}
    </div>
  );
}
