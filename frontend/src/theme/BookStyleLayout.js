/**
 * Custom Docusaurus theme component for book-style layout
 * Implements the book-style reading experience for Docusaurus pages
 */

import React from 'react';
import Layout from '@theme/Layout';
import BookLayout from '../components/BookLayout';

// Wrapper component that applies book-style layout to Docusaurus content
const BookStyleLayout = (props) => {
  const { children, title, description } = props;

  // Extract section information from the route if available
  const section = props.section || '';

  return (
    <Layout title={title} description={description}>
      <BookLayout title={title} section={section}>
        {children}
      </BookLayout>
    </Layout>
  );
};

export default BookStyleLayout;