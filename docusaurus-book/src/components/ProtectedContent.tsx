import React from 'react';
import { useAuth } from '../context/AuthContext';
import styles from './ProtectedContent.module.css';

interface ProtectedContentProps {
  children: React.ReactNode;
}

export default function ProtectedContent({ children }: ProtectedContentProps) {
  const { isAuthenticated, isPending, setShowLoginModal } = useAuth();

  // Show loading while checking auth
  if (isPending) {
    return (
      <div className={styles.loadingContainer}>
        <div className={styles.spinner}></div>
        <p>Loading...</p>
      </div>
    );
  }

  // Show login prompt if not authenticated
  if (!isAuthenticated) {
    return (
      <div className={styles.protectedContainer}>
        <div className={styles.lockIcon}>🔒</div>
        <h2 className={styles.title}>Authentication Required</h2>
        <p className={styles.message}>
          Please sign in to access the Physical AI & Humanoid Robotics Textbook content.
        </p>
        <button
          className={styles.loginButton}
          onClick={() => setShowLoginModal(true)}
        >
          Sign In to Continue
        </button>
        <p className={styles.signupHint}>
          Don't have an account? Click Sign In and then create one!
        </p>
      </div>
    );
  }

  // User is authenticated, show content
  return <>{children}</>;
}
