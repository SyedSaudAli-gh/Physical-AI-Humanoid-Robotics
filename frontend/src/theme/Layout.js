import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '../components/ChatWidget/ChatWidget';

function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatWidget />
    </>
  );
}

export default Layout;