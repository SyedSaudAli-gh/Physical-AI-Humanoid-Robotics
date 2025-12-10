// src/theme/Root.js
import React from 'react';
import { UserProvider } from '../contexts/UserContext';

// Default implementation, that you can customize
function Root({ children }) {
  return <UserProvider>{children}</UserProvider>;
}

export default Root;