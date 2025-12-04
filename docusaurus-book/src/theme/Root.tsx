import React from 'react';
import { useLocation } from '@docusaurus/router';
import { AuthProvider, useAuth } from '../context/AuthContext';
import AuthButton from '../components/AuthButton';
import ProtectedContent from '../components/ProtectedContent';
import styles from './Root.module.css';

// Lazy load Chatbot only on client side
const Chatbot = React.lazy(() => import('../components/Chatbot'));

function AppContent({ children }) {
  const location = useLocation();
  const { isAuthenticated } = useAuth();
  
  // Check if current route needs protection (book content)
  const isProtectedRoute = location.pathname.includes('/physical-ai-textbook');

  return (
    <>
      {isProtectedRoute ? (
        <ProtectedContent>{children}</ProtectedContent>
      ) : (
        children
      )}
      
      {/* Only show chatbot if authenticated */}
      {isAuthenticated && (
        <React.Suspense fallback={null}>
          <Chatbot />
        </React.Suspense>
      )}
      
      <div className={styles.authButtonContainer}>
        <AuthButton />
      </div>
    </>
  );
}

export default function Root({ children }) {
  return (
    <AuthProvider>
      <AppContent>{children}</AppContent>
    </AuthProvider>
  );
}
