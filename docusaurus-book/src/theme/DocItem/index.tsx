import React from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import { useLocation } from '@docusaurus/router';
import BrowserOnly from '@docusaurus/BrowserOnly';

type Props = WrapperProps<typeof DocItemType>;

// Lazy load components that use auth context
const ChapterToolsWrapper = React.lazy(() => import('../../components/ChapterToolsWrapper'));

export default function DocItemWrapper(props: Props): JSX.Element {
  const location = useLocation();
  
  // Extract chapter title from props or URL
  const chapterTitle = (props.content as any)?.metadata?.title || 
    location.pathname.split('/').pop()?.replace(/-/g, ' ') || 'Chapter';
  
  const chapterPath = location.pathname;

  return (
    <div className="doc-item-with-tools">
      {/* Interactive features only in browser - rendered above the doc content */}
      <BrowserOnly fallback={null}>
        {() => (
          <React.Suspense fallback={<div style={{padding: '1rem', textAlign: 'center', color: '#6366f1'}}>Loading chapter tools...</div>}>
            <div style={{ marginBottom: '1rem' }}>
              <ChapterToolsWrapper 
                chapterTitle={chapterTitle}
                chapterPath={chapterPath}
              />
            </div>
          </React.Suspense>
        )}
      </BrowserOnly>
      
      {/* Original Doc Item */}
      <DocItem {...props} />
    </div>
  );
}
