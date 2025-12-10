import React from 'react';
import OriginalNavbar from '@theme-original/Navbar';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';

const NavbarWrapper = (props) => {
  const { user, logout, isAuthenticated } = useAuth();
  const history = useHistory();

  const handleLogout = async () => {
    await logout();
    // Redirect to home after logout
    history.push('/');
    // Refresh the page to update the UI
    window.location.reload();
  };

  // Return the original navbar as-is, since the configuration in docusaurus.config.js
  // already has the login/logout links, and we'll handle the dynamic display with CSS
  return (
    <OriginalNavbar {...props} />
  );
};

export default NavbarWrapper;