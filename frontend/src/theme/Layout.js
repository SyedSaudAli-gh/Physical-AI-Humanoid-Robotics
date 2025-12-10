import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChapterControls from '../components/ChapterControls';
import { useLocation } from '@docusaurus/router';

// This layout will be used for documentation pages to add chapter controls
const ChapterLayout = (props) => {
  const {pathname} = useLocation();
  const isDocsRoute = pathname.startsWith('/docs/');

  // Extract chapter ID from the URL (assuming format like /docs/section/chapter-slug)
  const pathParts = pathname.split('/');
  const chapterSlug = pathParts.length > 2 ? pathParts[pathParts.length - 1] : '';

  const chapterId = chapterSlug || '1'; // fallback ID if no slug found

  return (
    <>
      {isDocsRoute && (
        <ChapterControls chapterId={chapterId} />
      )}
      <OriginalLayout {...props} />
    </>
  );
};

export default ChapterLayout;