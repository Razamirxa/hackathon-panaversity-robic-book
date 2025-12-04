import React, { useState, useEffect, useCallback, useRef } from 'react';
import { useAuth } from '../context/AuthContext';
import { contentAPI } from '../services/content_api';
import styles from './ReadingProgress.module.css';

interface ReadingProgressProps {
  chapterPath: string;
  chapterTitle: string;
}

export default function ReadingProgress({ chapterPath, chapterTitle }: ReadingProgressProps) {
  const { user, isAuthenticated } = useAuth();
  const [progress, setProgress] = useState(0);
  const [isCompleted, setIsCompleted] = useState(false);
  const [timeSpent, setTimeSpent] = useState(0);
  const startTimeRef = useRef<number>(Date.now());
  const lastSavedProgressRef = useRef<number>(0);
  const saveTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const isSavingRef = useRef<boolean>(false);

  // Track scroll progress (no API calls here)
  const updateScrollProgress = useCallback(() => {
    const scrollHeight = document.documentElement.scrollHeight - window.innerHeight;
    if (scrollHeight <= 0) return;
    
    const scrolled = window.scrollY;
    const percentage = Math.min(100, Math.round((scrolled / scrollHeight) * 100));
    setProgress(percentage);

    if (percentage >= 90 && !isCompleted) {
      setIsCompleted(true);
    }
  }, [isCompleted]);

  // Save progress to database - only called on specific events, not on scroll
  const saveProgressToServer = useCallback(async (forceUpdate = false) => {
    if (!isAuthenticated || !user?.id || isSavingRef.current) return;
    
    // Only save if progress changed significantly (more than 10%) or forced
    const progressDiff = Math.abs(progress - lastSavedProgressRef.current);
    if (!forceUpdate && progressDiff < 10) return;

    isSavingRef.current = true;
    const sessionTime = Math.round((Date.now() - startTimeRef.current) / 1000);

    try {
      await contentAPI.updateProgress({
        user_id: user.id,
        chapter_path: chapterPath,
        chapter_title: chapterTitle,
        scroll_position: progress,
        time_spent_seconds: sessionTime,
        is_completed: isCompleted,
      });
      lastSavedProgressRef.current = progress;
    } catch (error) {
      console.error('Failed to save progress:', error);
    } finally {
      isSavingRef.current = false;
    }
  }, [isAuthenticated, user, chapterPath, chapterTitle, progress, isCompleted]);

  // Throttled scroll listener
  useEffect(() => {
    let ticking = false;
    
    const handleScroll = () => {
      if (!ticking) {
        window.requestAnimationFrame(() => {
          updateScrollProgress();
          ticking = false;
        });
        ticking = true;
      }
    };

    window.addEventListener('scroll', handleScroll, { passive: true });
    updateScrollProgress(); // Initial check

    return () => {
      window.removeEventListener('scroll', handleScroll);
    };
  }, [updateScrollProgress]);

  // Save progress every 60 seconds (not on scroll)
  useEffect(() => {
    const interval = setInterval(() => {
      saveProgressToServer(false);
    }, 60000); // Every 60 seconds

    return () => {
      clearInterval(interval);
    };
  }, [saveProgressToServer]);

  // Save on unmount or page leave
  useEffect(() => {
    const handleBeforeUnload = () => {
      // Use sendBeacon for reliable save on page leave
      if (isAuthenticated && user?.id) {
        const sessionTime = Math.round((Date.now() - startTimeRef.current) / 1000);
        const data = JSON.stringify({
          user_id: user.id,
          chapter_path: chapterPath,
          chapter_title: chapterTitle,
          scroll_position: progress,
          time_spent_seconds: sessionTime,
          is_completed: isCompleted,
        });
        navigator.sendBeacon('/api/progress/update', data);
      }
    };

    window.addEventListener('beforeunload', handleBeforeUnload);
    
    return () => {
      window.removeEventListener('beforeunload', handleBeforeUnload);
      // Save on component unmount
      saveProgressToServer(true);
    };
  }, [isAuthenticated, user, chapterPath, chapterTitle, progress, isCompleted, saveProgressToServer]);

  // Track time spent (local only, no API calls)
  useEffect(() => {
    const interval = setInterval(() => {
      setTimeSpent(Math.round((Date.now() - startTimeRef.current) / 1000));
    }, 1000);

    return () => clearInterval(interval);
  }, []);

  const formatTime = (seconds: number): string => {
    const mins = Math.floor(seconds / 60);
    const secs = seconds % 60;
    return mins > 0 ? `${mins}m ${secs}s` : `${secs}s`;
  };

  const handleMarkComplete = async () => {
    if (!isAuthenticated || !user?.id) return;

    try {
      await contentAPI.markComplete(user.id, chapterPath, chapterTitle);
      setIsCompleted(true);
      setProgress(100);
    } catch (error) {
      console.error('Failed to mark complete:', error);
    }
  };

  if (!isAuthenticated) return null;

  return (
    <div className={styles.progressContainer}>
      {/* Progress Bar */}
      <div className={styles.progressBar}>
        <div
          className={styles.progressFill}
          style={{ width: `${progress}%` }}
        />
      </div>

      {/* Stats */}
      <div className={styles.progressStats}>
        <span className={styles.stat}>
          📖 {progress}% read
        </span>
        <span className={styles.stat}>
          ⏱️ {formatTime(timeSpent)}
        </span>
        {isCompleted ? (
          <span className={styles.completedBadge}>
            ✅ Completed
          </span>
        ) : (
          <button
            className={styles.completeButton}
            onClick={handleMarkComplete}
          >
            Mark Complete
          </button>
        )}
      </div>
    </div>
  );
}
