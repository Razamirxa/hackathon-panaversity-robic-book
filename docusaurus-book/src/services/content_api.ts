/**
 * Content API Service
 * Handles personalization, translation, summarization, and explanation
 */

import { API_BASE_URL } from './api-config';

// Types
export interface UserProfile {
  hardware_type?: string;
  hardware_specs?: string;
  has_gpu?: boolean;
  has_robot_kit?: boolean;
  robot_kit_type?: string;
  coding_experience?: string;
  robotics_experience?: string;
  ros_experience?: string;
  ml_experience?: string;
  learning_goals?: string[];
  areas_of_interest?: string[];
  preferred_pace?: string;
}

export interface PersonalizeRequest {
  content: string;
  chapter_title?: string;
  user_id?: number;
  user_profile?: UserProfile;
}

export interface PersonalizeResponse {
  success: boolean;
  personalized_content: string;
  adaptations?: {
    hardware_type: string;
    experience_level: string;
    tone: string;
  };
  error?: string;
}

export interface TranslateRequest {
  content: string;
  target_language?: string;
  preserve_code?: boolean;
}

export interface TranslateResponse {
  success: boolean;
  translated_content: string;
  target_language: string;
  error?: string;
}

export interface SummarizeRequest {
  content: string;
  summary_type?: 'brief' | 'detailed' | 'bullet_points';
  user_level?: string;
}

export interface SummarizeResponse {
  success: boolean;
  summary: string;
  summary_type: string;
  error?: string;
}

export interface ExplainRequest {
  selected_text: string;
  chapter_context?: string;
  question?: string;
  user_id?: number;
}

export interface ExplainResponse {
  success: boolean;
  explanation: string;
  user_level: string;
  error?: string;
}

// Progress Tracking Types
export interface ProgressUpdate {
  user_id: number;
  chapter_path: string;
  chapter_title?: string;
  scroll_position?: number;
  time_spent_seconds?: number;
  is_completed?: boolean;
}

export interface ChapterProgress {
  id: number;
  chapter_path: string;
  chapter_title?: string;
  is_completed: boolean;
  completion_percentage: number;
  scroll_position: number;
  time_spent_seconds: number;
  first_visited: string;
  last_visited: string;
  completed_at?: string;
}

export interface UserProgressSummary {
  total_chapters_visited: number;
  total_chapters_completed: number;
  total_time_spent_seconds: number;
  completion_percentage: number;
  chapters: ChapterProgress[];
}

// API Client Class
class ContentAPI {
  private baseURL: string;

  constructor(baseURL: string = API_BASE_URL) {
    this.baseURL = baseURL;
  }

  /**
   * Personalize content based on user profile
   */
  async personalize(request: PersonalizeRequest): Promise<PersonalizeResponse> {
    const response = await fetch(`${this.baseURL}/content/personalize`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Personalization failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Translate content to target language (default: Urdu)
   */
  async translate(request: TranslateRequest): Promise<TranslateResponse> {
    const response = await fetch(`${this.baseURL}/content/translate`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({
        content: request.content,
        target_language: request.target_language || 'Urdu',
        preserve_code: request.preserve_code ?? true,
      }),
    });

    if (!response.ok) {
      throw new Error(`Translation failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Summarize content
   */
  async summarize(request: SummarizeRequest): Promise<SummarizeResponse> {
    const response = await fetch(`${this.baseURL}/content/summarize`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify({
        content: request.content,
        summary_type: request.summary_type || 'brief',
        user_level: request.user_level || 'intermediate',
      }),
    });

    if (!response.ok) {
      throw new Error(`Summarization failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Explain selected text
   */
  async explain(request: ExplainRequest): Promise<ExplainResponse> {
    const response = await fetch(`${this.baseURL}/content/explain`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Explanation failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Update reading progress
   */
  async updateProgress(progress: ProgressUpdate): Promise<ChapterProgress> {
    const response = await fetch(`${this.baseURL}/progress/update`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      credentials: 'include',
      body: JSON.stringify(progress),
    });

    if (!response.ok) {
      throw new Error(`Progress update failed: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Get user's overall progress
   */
  async getUserProgress(userId: number): Promise<UserProgressSummary> {
    const response = await fetch(`${this.baseURL}/progress/user/${userId}`, {
      credentials: 'include',
    });

    if (!response.ok) {
      throw new Error(`Failed to get progress: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Get progress for specific chapter
   */
  async getChapterProgress(userId: number, chapterPath: string): Promise<ChapterProgress | null> {
    const response = await fetch(
      `${this.baseURL}/progress/chapter/${userId}?chapter_path=${encodeURIComponent(chapterPath)}`,
      { credentials: 'include' }
    );

    if (!response.ok) {
      if (response.status === 404) return null;
      throw new Error(`Failed to get chapter progress: ${response.statusText}`);
    }

    return response.json();
  }

  /**
   * Mark chapter as complete
   */
  async markComplete(userId: number, chapterPath: string, chapterTitle?: string): Promise<void> {
    const params = new URLSearchParams({ chapter_path: chapterPath });
    if (chapterTitle) params.append('chapter_title', chapterTitle);

    const response = await fetch(
      `${this.baseURL}/progress/mark-complete/${userId}?${params.toString()}`,
      {
        method: 'POST',
        credentials: 'include',
      }
    );

    if (!response.ok) {
      throw new Error(`Failed to mark complete: ${response.statusText}`);
    }
  }
}

export const contentAPI = new ContentAPI();
