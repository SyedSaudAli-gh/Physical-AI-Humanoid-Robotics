import React from 'react';
import { useAuth } from '../contexts/AuthContext';

// Authentication-aware Navbar Item Component
const AuthNavbarItem = () => {
  const { user, isAuthenticated, loading, logout } = useAuth();

  if (loading) {
    return (
      <div className="navbar__item navbar__link">
        <span>Loading...</span>
      </div>
    );
  }

  if (isAuthenticated && user) {
    return (
      <div className="navbar__item dropdown dropdown--hoverable">
        <span className="navbar__link">
          {user.name || user.email} ▾
        </span>
        <ul className="dropdown__menu">
          <li>
            <a className="dropdown__link" href="/user-profile">
              Profile
            </a>
          </li>
          <li>
            <a className="dropdown__link" href="/settings">
              Settings
            </a>
          </li>
          <li>
            <button 
              className="dropdown__link" 
              onClick={logout}
              style={{ width: '100%', textAlign: 'left', cursor: 'pointer', border: 'none', background: 'none' }}
            >
              Logout
            </button>
          </li>
        </ul>
      </div>
    );
  } else {
    return (
      <div className="navbar__item dropdown dropdown--hoverable">
        <span className="navbar__link">
          👤 ▾
        </span>
        <ul className="dropdown__menu">
          <li>
            <a className="dropdown__link" href="/login">
              Login
            </a>
          </li>
          <li>
            <a className="dropdown__link" href="/signup">
              Sign Up
            </a>
          </li>
        </ul>
      </div>
    );
  }
};

export default AuthNavbarItem;