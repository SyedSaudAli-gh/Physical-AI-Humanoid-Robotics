import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';

const LoginForm = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const { login } = useAuth();

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!window) {
      setError('Login is only available in the browser');
      return;
    }

    setLoading(true);
    setError('');

    try {
      await login(email, password);
      // On successful login, redirect to home or previous page
      window.location.href = '/';
    } catch (err) {
      setError(err.message || 'Login failed. Please check your credentials and try again.');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="auth-form-container">
      <div className="auth-form-step-indicator">
        <h2>Login</h2>
        <p>Access your account</p>
      </div>

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
            placeholder="Enter your email"
          />
        </div>

        <div className="form-group margin-bottom--md">
          <label htmlFor="password" className="form-label">Password</label>
          <input
            type="password"
            className="form-control"
            id="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            placeholder="Enter your password"
          />
        </div>

        <div className="margin-top--lg">
          <button type="submit" className="button button--primary button--lg button--block" disabled={loading}>
            {loading ? 'Logging in...' : 'Login'}
          </button>
        </div>

        {error && (
          <div className="alert alert--danger margin-top--md" role="alert">
            {error}
          </div>
        )}
      </form>

      <div className="text--center margin-top--lg">
        <p>Don't have an account? <a href="/signup">Sign up here</a></p>
      </div>
    </div>
  );
};

export default LoginForm;