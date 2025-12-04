/**
 * API Configuration
 * Centralized API URL configuration for all services
 */

// Get API URL from Docusaurus config or environment
const getApiUrl = (): string => {
  // Check for Docusaurus site config (available in browser)
  if (typeof window !== 'undefined') {
    try {
      // Try to get from Docusaurus config
      const docusaurusConfig = (window as any).__DOCUSAURUS__;
      if (docusaurusConfig?.siteConfig?.customFields?.apiUrl) {
        return docusaurusConfig.siteConfig.customFields.apiUrl;
      }
    } catch (e) {
      // Ignore errors
    }
    
    // Check for window variable (can be set at runtime)
    if ((window as any).__API_URL__) {
      return (window as any).__API_URL__;
    }
  }
  
  // Check for environment variable (works in build time for Node.js context)
  if (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL) {
    return process.env.REACT_APP_API_URL;
  }
  
  // Default to localhost for development
  return 'http://localhost:8000/api';
};

export const API_BASE_URL = getApiUrl();

// Helper to make API calls with proper error handling
export async function apiRequest<T>(
  endpoint: string,
  options: RequestInit = {}
): Promise<T> {
  const url = `${API_BASE_URL}${endpoint}`;
  
  const defaultHeaders: HeadersInit = {
    'Content-Type': 'application/json',
  };

  const response = await fetch(url, {
    ...options,
    headers: {
      ...defaultHeaders,
      ...options.headers,
    },
    credentials: 'include', // Include cookies for auth
  });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({}));
    throw new Error(errorData.detail || `API error: ${response.status}`);
  }

  return response.json();
}

export default API_BASE_URL;
