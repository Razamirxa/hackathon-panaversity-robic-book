import React, { createContext, useContext, useState, useEffect, useCallback, ReactNode } from 'react';
import { useSession, User, SessionData } from '../lib/auth-client';

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isPending: boolean;
  needsOnboarding: boolean;
  showLoginModal: boolean;
  setShowLoginModal: (show: boolean) => void;
  refetch: () => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: ReactNode }) {
  const { data, isPending, refetch } = useSession();
  const [showLoginModal, setShowLoginModal] = useState(false);

  return (
    <AuthContext.Provider
      value={{
        user: data.user,
        isAuthenticated: !!data.user,
        isPending,
        needsOnboarding: data.needsOnboarding,
        showLoginModal,
        setShowLoginModal,
        refetch,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
