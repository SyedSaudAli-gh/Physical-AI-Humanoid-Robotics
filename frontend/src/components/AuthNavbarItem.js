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
      <div className="navbar__item dropdown dropdown--hoverable dropdown--right">
        <span className="navbar__link navbar__link--icon">
          <span className="avatar">
            <span className="avatar__name">{user.name?.charAt(0) || user.email?.charAt(0)}</span>
          </span>
          <span className="dropdown__label">{user.name || user.email.split('@')[0]}</span>
          <span className="dropdown__caret">▾</span>
        </span>
        <ul className="dropdown__menu">
          <li>
            <a className="dropdown__link" href="/user-profile">
              <i className="fa fa-user"></i> Profile
            </a>
          </li>
          <li>
            <a className="dropdown__link" href="/user-settings">
              <i className="fa fa-cog"></i> Settings
            </a>
          </li>
          <li>
            <button
              className="dropdown__link"
              onClick={logout}
              style={{ width: '100%', textAlign: 'left', cursor: 'pointer', border: 'none', background: 'none' }}
            >
              <i className="fa fa-sign-out"></i> Logout
            </button>
          </li>
        </ul>
      </div>
    );
  } else {
    return (
      <div className="navbar__item dropdown dropdown--hoverable dropdown--right">
        <span className="navbar__link navbar__link--icon">
          <span className="dropdown__label">👤 Account</span>
          <span className="dropdown__caret">▾</span>
        </span>
        <ul className="dropdown__menu">
          <li>
            <a className="dropdown__link" href="/login">
              <i className="fa fa-sign-in"></i> Login
            </a>
          </li>
          <li>
            <a className="dropdown__link" href="/signup">
              <i className="fa fa-user-plus"></i> Sign Up
            </a>
          </li>
        </ul>
      </div>
    );
  }
};

export default AuthNavbarItem;