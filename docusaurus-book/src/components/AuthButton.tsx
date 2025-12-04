import React, { useState, useEffect } from 'react';
import { useSession, markOnboardingComplete } from '../lib/auth-client';
import { useAuth } from '../context/AuthContext';
import AuthModal from './AuthModal';
import OnboardingModal from './OnboardingModal';
import styles from './AuthButton.module.css';

export default function AuthButton() {
  const [isOnboardingOpen, setIsOnboardingOpen] = useState(false);
  const { data: session, isPending, refetch } = useSession();
  const { showLoginModal, setShowLoginModal } = useAuth();

  // Show onboarding modal if user needs it
  useEffect(() => {
    if (session?.user && session.needsOnboarding) {
      setIsOnboardingOpen(true);
    }
  }, [session]);

  const handleOnboardingComplete = () => {
    markOnboardingComplete();
    setIsOnboardingOpen(false);
    refetch(); // Refresh session data
  };

  const handleAuthSuccess = (needsOnboarding: boolean) => {
    setShowLoginModal(false);
    if (needsOnboarding) {
      setIsOnboardingOpen(true);
    }
  };

  if (isPending) {
    return <div className={styles.button}>...</div>;
  }

  return (
    <>
      <button 
        className={styles.button}
        onClick={() => setShowLoginModal(true)}
      >
        {session?.user ? (
          <span className={styles.userButton}>
            <span className={styles.avatar}>
              {session.user.name?.charAt(0).toUpperCase() || '?'}
            </span>
            <span className={styles.userName}>{session.user.name}</span>
          </span>
        ) : (
          'Sign In'
        )}
      </button>
      
      <AuthModal 
        isOpen={showLoginModal} 
        onClose={() => setShowLoginModal(false)}
        onAuthSuccess={handleAuthSuccess}
      />

      <OnboardingModal
        isOpen={isOnboardingOpen}
        onComplete={handleOnboardingComplete}
        userName={session?.user?.name}
      />
    </>
  );
}
