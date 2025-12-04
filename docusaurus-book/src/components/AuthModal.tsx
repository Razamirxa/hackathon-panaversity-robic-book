import React, { useState } from 'react';
import { signIn, signUp, useSession, signOut } from '../lib/auth-client';
import styles from './AuthModal.module.css';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  onAuthSuccess?: (needsOnboarding: boolean) => void;
}

export default function AuthModal({ isOpen, onClose, onAuthSuccess }: AuthModalProps) {
  const [isLogin, setIsLogin] = useState(true);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const { data: session } = useSession();

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      if (isLogin) {
        const result = await signIn(email, password);
        onAuthSuccess?.(result.needsOnboarding);
        onClose();
      } else {
        const result = await signUp(email, password, name);
        onAuthSuccess?.(result.needsOnboarding);
        onClose();
      }
    } catch (err: any) {
      setError(err.message || 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  const handleSignOut = async () => {
    await signOut();
    onClose();
  };

  if (!isOpen) return null;

  // If user is logged in, show account info
  if (session?.user) {
    return (
      <div className={styles.overlay} onClick={onClose}>
        <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
          <button className={styles.closeButton} onClick={onClose}>×</button>
          <h2 className={styles.title}>Your Account</h2>
          <div className={styles.userInfo}>
            <p><strong>Name:</strong> {session.user.name}</p>
            <p><strong>Email:</strong> {session.user.email}</p>
            <p><strong>Status:</strong> {session.user.onboardingCompleted ? '✅ Profile Complete' : '⏳ Profile Incomplete'}</p>
          </div>
          <button 
            className={styles.signOutButton} 
            onClick={handleSignOut}
          >
            Sign Out
          </button>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={onClose}>×</button>
        
        <h2 className={styles.title}>{isLogin ? 'Sign In' : 'Create Account'}</h2>
        
        {error && <div className={styles.error}>{error}</div>}
        
        <form onSubmit={handleSubmit} className={styles.form}>
          {!isLogin && (
            <div className={styles.inputGroup}>
              <label htmlFor="name">Name</label>
              <input
                type="text"
                id="name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                placeholder="Your name"
                required={!isLogin}
              />
            </div>
          )}
          
          <div className={styles.inputGroup}>
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="you@example.com"
              required
            />
          </div>
          
          <div className={styles.inputGroup}>
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder="••••••••"
              required
              minLength={8}
            />
            {!isLogin && <small style={{ color: 'var(--ifm-color-emphasis-600)', marginTop: '4px' }}>Minimum 8 characters</small>}
          </div>
          
          <button 
            type="submit" 
            className={styles.submitButton}
            disabled={loading}
          >
            {loading ? 'Loading...' : (isLogin ? 'Sign In' : 'Create Account')}
          </button>
        </form>
        
        <div className={styles.switchMode}>
          {isLogin ? (
            <p>
              Don't have an account?{' '}
              <button onClick={() => { setIsLogin(false); setError(''); }}>Sign Up</button>
            </p>
          ) : (
            <p>
              Already have an account?{' '}
              <button onClick={() => { setIsLogin(true); setError(''); }}>Sign In</button>
            </p>
          )}
        </div>
      </div>
    </div>
  );
}
