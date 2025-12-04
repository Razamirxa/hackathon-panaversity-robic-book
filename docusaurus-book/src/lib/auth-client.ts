import { useState, useEffect, useCallback } from 'react';
import { API_BASE_URL } from '../services/api-config';

const API_BASE = API_BASE_URL.replace('/api', '/api'); // Use shared config

// User type
export interface User {
  id: number;
  email: string;
  name: string;
  createdAt: string;
  onboardingCompleted: boolean;
}

// Session response type
export interface SessionData {
  user: User | null;
  needsOnboarding: boolean;
}

// Auth state store (simple React-friendly implementation)
let authState: SessionData = {
  user: null,
  needsOnboarding: false,
};

let listeners: Set<() => void> = new Set();

function notifyListeners() {
  listeners.forEach((listener) => listener());
}

function setAuthState(newState: Partial<SessionData>) {
  authState = { ...authState, ...newState };
  notifyListeners();
}

// Sign up with email/password
export async function signUp(email: string, password: string, name: string): Promise<{ user: User; needsOnboarding: boolean }> {
  const response = await fetch(`${API_BASE}/auth/sign-up/email`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    credentials: 'include',
    body: JSON.stringify({ email, password, name }),
  });

  if (!response.ok) {
    const data = await response.json();
    throw new Error(data.detail || 'Sign up failed');
  }

  const data = await response.json();
  setAuthState({ user: data.user, needsOnboarding: data.needsOnboarding });
  return { user: data.user, needsOnboarding: data.needsOnboarding };
}

// Sign in with email/password
export async function signIn(email: string, password: string, rememberMe = true): Promise<{ user: User; needsOnboarding: boolean }> {
  const response = await fetch(`${API_BASE}/auth/sign-in/email`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    credentials: 'include',
    body: JSON.stringify({ email, password, rememberMe }),
  });

  if (!response.ok) {
    const data = await response.json();
    throw new Error(data.detail || 'Sign in failed');
  }

  const data = await response.json();
  setAuthState({ user: data.user, needsOnboarding: data.needsOnboarding });
  return { user: data.user, needsOnboarding: data.needsOnboarding };
}

// Sign out
export async function signOut(): Promise<void> {
  await fetch(`${API_BASE}/auth/sign-out`, {
    method: 'POST',
    credentials: 'include',
  });
  setAuthState({ user: null, needsOnboarding: false });
}

// Get current session
export async function getSession(): Promise<SessionData> {
  try {
    const response = await fetch(`${API_BASE}/auth/session`, {
      credentials: 'include',
    });

    if (!response.ok) {
      return { user: null, needsOnboarding: false };
    }

    const data = await response.json();
    setAuthState(data);
    return data;
  } catch {
    return { user: null, needsOnboarding: false };
  }
}

// Mark onboarding as complete
export function markOnboardingComplete() {
  if (authState.user) {
    setAuthState({
      user: { ...authState.user, onboardingCompleted: true },
      needsOnboarding: false,
    });
  }
}

// React hook to use session
export function useSession(): {
  data: SessionData;
  isPending: boolean;
  refetch: () => Promise<void>;
} {
  const [data, setData] = useState<SessionData>(authState);
  const [isPending, setIsPending] = useState(true);

  const refetch = useCallback(async () => {
    setIsPending(true);
    await getSession();
    setIsPending(false);
  }, []);

  useEffect(() => {
    // Subscribe to state changes
    const listener = () => {
      setData({ ...authState });
    };
    listeners.add(listener);

    // Initial fetch
    refetch();

    return () => {
      listeners.delete(listener);
    };
  }, [refetch]);

  return { data, isPending, refetch };
}
