import React, { createContext, useContext, useState, useEffect } from 'react';
import axios from 'axios';

// Create context
const AuthContext = createContext(undefined);

// API base URL - using relative path for proxy compatibility
// This will be proxied to the backend during development
const API_BASE_URL = '/api';

// Set up axios defaults
axios.defaults.withCredentials = true;

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);

  // Check for existing session on mount
  useEffect(() => {
    const checkSession = async () => {
      try {
        // Try to get user info to verify session
        const token = localStorage.getItem('auth_token');
        if (token) {
          // If we have a token, verify it by making a request to the backend
          try {
            const response = await axios.get(`${API_BASE_URL}/auth/me`, {
              headers: {
                'Authorization': `Bearer ${token}`
              }
            });
            if (response.data && response.data.user) {
              setUser(response.data.user);
            }
          } catch (verifyError) {
            // If token verification fails, clear it
            localStorage.removeItem('auth_token');
            localStorage.removeItem('user');
            console.error('Token verification failed:', verifyError);
          }
        }
      } catch (error) {
        console.error('Error checking session:', error);
        // Clear any stored auth data if verification fails
        localStorage.removeItem('auth_token');
        localStorage.removeItem('user');
      } finally {
        setLoading(false);
      }
    };

    checkSession();
  }, []);

  const login = async (email, password) => {
    try {
      // Call the backend login endpoint
      const response = await axios.post(`${API_BASE_URL}/auth/login`, {
        email,
        password
      });

      // Store token from response
      if (response.data.access_token) {
        localStorage.setItem('auth_token', response.data.access_token);

        // Now fetch user details using the token
        const userResponse = await axios.get(`${API_BASE_URL}/auth/me`, {
          headers: {
            'Authorization': `Bearer ${response.data.access_token}`
          }
        });

        if (userResponse.data && userResponse.data.user) {
          localStorage.setItem('user', JSON.stringify(userResponse.data.user));
          setUser(userResponse.data.user);
        }
      }

      return response.data;
    } catch (error) {
      console.error('Login error:', error);
      if (error.response) {
        // Server responded with error status
        console.error('Error details:', error.response.data);
        throw new Error(error.response.data.detail || 'Login failed');
      } else if (error.request) {
        // Request was made but no response received
        throw new Error('Network error - could not reach authentication server');
      } else {
        // Something else happened
        throw new Error('An unexpected error occurred during login');
      }
    }
  };

  const signup = async (userData) => {
    try {
      // Call the backend signup endpoint
      const response = await axios.post(`${API_BASE_URL}/auth/signup`, userData);

      // Store token and user information after successful signup
      if (response.data.access_token) {
        localStorage.setItem('auth_token', response.data.access_token);
      }
      if (response.data.user) {
        localStorage.setItem('user', JSON.stringify(response.data.user));
        setUser(response.data.user);
      }

      return response.data;
    } catch (error) {
      console.error('Signup error:', error);
      throw error;
    }
  };

  const logout = () => {
    // Clear stored authentication data
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user');
    setUser(null);
  };

  const isAuthenticated = !!user;

  const value = {
    user,
    loading,
    login,
    signup,
    logout,
    isAuthenticated
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};