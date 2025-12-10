import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';
import { useHistory, Redirect } from '@docusaurus/router';

const LoginPage = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { login, isAuthenticated } = useAuth();
  const history = useHistory();

  if (isAuthenticated) {
    return <Redirect to="/" />;
  }

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      await login(email, password);
      // On successful login, redirect to previous location or home
      const returnUrl = new URLSearchParams(window.location.search).get('returnUrl') || '/';
      history.push(returnUrl);
    } catch (err) {
      setError(err.message || 'Login failed. Please check your credentials and try again.');
    }
  };

  return (
    <div className="container margin-vert--xl">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <div className="card">
            <div className="card__header text-center">
              <h2>Login to Physical AI & Humanoid Robotics Textbook</h2>
            </div>
            <div className="card__body">
              {error && (
                <div className="alert alert--danger" role="alert">
                  {error}
                </div>
              )}

              <form onSubmit={handleSubmit}>
                <div className="form-group margin-bottom--md">
                  <label htmlFor="email" className="form-label">Email</label>
                  <input
                    type="email"
                    className="form-control"
                    id="email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    required
                    autoComplete="username"
                  />
                </div>

                <div className="form-group margin-bottom--lg">
                  <label htmlFor="password" className="form-label">Password</label>
                  <input
                    type="password"
                    className="form-control"
                    id="password"
                    value={password}
                    onChange={(e) => setPassword(e.target.value)}
                    required
                    autoComplete="current-password"
                  />
                </div>

                <div className="form-group margin-bottom--md">
                  <button type="submit" className="button button--primary button--block">
                    Login
                  </button>
                </div>
              </form>

              <div className="text--center">
                <p>Don't have an account? <a href="/signup">Sign up here</a></p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default LoginPage;