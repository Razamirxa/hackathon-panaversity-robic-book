import React, { useState, useEffect, useCallback, useRef } from 'react';
import { contentAPI } from '../services/content_api';
import { useAuth } from '../context/AuthContext';
import styles from './TextSelectionMenu.module.css';

interface TextSelectionMenuProps {
  chapterContext?: string;
}

export default function TextSelectionMenu({ chapterContext = '' }: TextSelectionMenuProps) {
  const { user } = useAuth();
  const [selectedText, setSelectedText] = useState('');
  const [menuPosition, setMenuPosition] = useState<{ x: number; y: number } | null>(null);
  const [showResult, setShowResult] = useState(false);
  const [resultContent, setResultContent] = useState('');
  const [resultType, setResultType] = useState<'explain' | 'summarize' | 'translate'>('explain');
  const [isLoading, setIsLoading] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);
  const resultRef = useRef<HTMLDivElement>(null);

  // Handle text selection
  const handleSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim() || '';

    if (text && text.length > 10) {
      setSelectedText(text);

      // Get position of selection (use viewport coordinates for fixed positioning)
      const range = selection?.getRangeAt(0);
      if (range) {
        const rect = range.getBoundingClientRect();
        setMenuPosition({
          x: rect.left + rect.width / 2,
          y: rect.bottom + 10, // No need for scrollY with fixed positioning
        });
        console.log('[TextSelectionMenu] Menu showing at:', rect.left + rect.width / 2, rect.bottom + 10);
      }
    } else {
      // Don't immediately hide - check if clicking inside menu/result
      setTimeout(() => {
        const activeElement = document.activeElement;
        if (!menuRef.current?.contains(activeElement) && !resultRef.current?.contains(activeElement)) {
          setMenuPosition(null);
        }
      }, 100);
    }
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('keyup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('keyup', handleSelection);
    };
  }, [handleSelection]);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (e: MouseEvent) => {
      if (
        menuRef.current && !menuRef.current.contains(e.target as Node) &&
        resultRef.current && !resultRef.current.contains(e.target as Node)
      ) {
        setMenuPosition(null);
        setShowResult(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleExplain = async () => {
    setIsLoading(true);
    setResultType('explain');
    setShowResult(true);
    setResultContent('');

    try {
      const response = await contentAPI.explain({
        selected_text: selectedText,
        chapter_context: chapterContext,
        user_id: user?.id,
      });

      if (response.success) {
        setResultContent(response.explanation);
      } else {
        setResultContent('Failed to generate explanation. Please try again.');
      }
    } catch (error) {
      setResultContent('Error: ' + (error instanceof Error ? error.message : 'Unknown error'));
    } finally {
      setIsLoading(false);
    }
  };

  const handleSummarize = async () => {
    setIsLoading(true);
    setResultType('summarize');
    setShowResult(true);
    setResultContent('');

    try {
      const response = await contentAPI.summarize({
        content: selectedText,
        summary_type: 'brief',
        user_level: 'intermediate',
      });

      if (response.success) {
        setResultContent(response.summary);
      } else {
        setResultContent('Failed to generate summary. Please try again.');
      }
    } catch (error) {
      setResultContent('Error: ' + (error instanceof Error ? error.message : 'Unknown error'));
    } finally {
      setIsLoading(false);
    }
  };

  const handleTranslate = async () => {
    setIsLoading(true);
    setResultType('translate');
    setShowResult(true);
    setResultContent('');

    try {
      const response = await contentAPI.translate({
        content: selectedText,
        target_language: 'Urdu',
        preserve_code: true,
      });

      if (response.success) {
        setResultContent(response.translated_content);
      } else {
        setResultContent('Failed to translate. Please try again.');
      }
    } catch (error) {
      setResultContent('Error: ' + (error instanceof Error ? error.message : 'Unknown error'));
    } finally {
      setIsLoading(false);
    }
  };

  const handleClose = () => {
    setMenuPosition(null);
    setShowResult(false);
    setSelectedText('');
  };

  if (!menuPosition) return null;

  return (
    <>
      {/* Selection Menu */}
      <div
        ref={menuRef}
        className={styles.selectionMenu}
        style={{
          left: `${menuPosition.x}px`,
          top: `${menuPosition.y}px`,
          transform: 'translateX(-50%)',
        }}
      >
        <button
          className={styles.menuButton}
          onClick={handleExplain}
          disabled={isLoading}
          title="Get AI explanation"
        >
          <span className={styles.buttonIcon}>💡</span>
          Explain
        </button>
        <button
          className={styles.menuButton}
          onClick={handleSummarize}
          disabled={isLoading}
          title="Summarize selected text"
        >
          <span className={styles.buttonIcon}>📝</span>
          Summarize
        </button>
        <button
          className={styles.menuButton}
          onClick={handleTranslate}
          disabled={isLoading}
          title="Translate to Urdu"
        >
          <span className={styles.buttonIcon}>🌐</span>
          Urdu
        </button>
      </div>

      {/* Result Panel */}
      {showResult && (
        <div
          ref={resultRef}
          className={styles.resultPanel}
          style={{
            left: `${menuPosition.x}px`,
            top: `${menuPosition.y + 50}px`,
            transform: 'translateX(-50%)',
          }}
        >
          <div className={styles.resultHeader}>
            <span className={styles.resultTitle}>
              {resultType === 'explain' && '💡 Explanation'}
              {resultType === 'summarize' && '📝 Summary'}
              {resultType === 'translate' && '🌐 Urdu Translation'}
            </span>
            <button className={styles.closeButton} onClick={handleClose}>
              ×
            </button>
          </div>
          <div className={styles.resultContent}>
            {isLoading ? (
              <div className={styles.loading}>
                <div className={styles.spinner}></div>
                <span>Processing...</span>
              </div>
            ) : (
              <div className={resultType === 'translate' ? styles.urduText : ''}>
                {resultContent}
              </div>
            )}
          </div>
          <div className={styles.selectedPreview}>
            <strong>Selected:</strong> "{selectedText.substring(0, 100)}
            {selectedText.length > 100 ? '...' : ''}"
          </div>
        </div>
      )}
    </>
  );
}
