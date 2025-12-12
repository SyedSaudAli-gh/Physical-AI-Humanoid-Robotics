import React from 'react';
import { useAuth } from '../../contexts/AuthContext';
import NavbarNavLink from '@theme/NavbarItem/NavbarNavLink';

const AuthNavbarItem = (props) => {
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
      <div className="navbar__item dropdown dropdown--right dropdown--is-clickable">
        <span className="navbar__link">
          {user.name || user.email.split('@')[0]} ▾
        </span>
        <ul className="dropdown__menu">
          <li>
            <NavbarNavLink href="/user-profile">
              Profile
            </NavbarNavLink>
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
      <div className="navbar__item dropdown dropdown--right dropdown--is-clickable">
        <span className="navbar__link">
          👤 ▾
        </span>
        <ul className="dropdown__menu">
          <li>
            <NavbarNavLink href="/login">
              Login
            </NavbarNavLink>
          </li>
          <li>
            <NavbarNavLink href="/signup">
              Sign Up
            </NavbarNavLink>
          </li>
        </ul>
      </div>
    );
  }
};

export default AuthNavbarItem;