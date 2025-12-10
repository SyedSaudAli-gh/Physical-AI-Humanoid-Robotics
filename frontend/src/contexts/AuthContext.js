import React, { createContext, useContext, useReducer, useEffect } from 'react';
import { useHistory, useLocation } from '@docusaurus/router';

// Auth context for handling user authentication across the Docusaurus site
const AuthContext = createContext();

// Initial state
const initialState = {
  user: null,
  token: null,
  isAuthenticated: false,
  loading: true,
  error: null
};

// Action types
const actionTypes = {
  LOGIN_START: 'LOGIN_START',
  LOGIN_SUCCESS: 'LOGIN_SUCCESS',
  LOGIN_FAILURE: 'LOGIN_FAILURE',
  LOGOUT: 'LOGOUT',
  CHECK_AUTH_START: 'CHECK_AUTH_START',
  CHECK_AUTH_SUCCESS: 'CHECK_AUTH_SUCCESS',
  CHECK_AUTH_FAILURE: 'CHECK_AUTH_FAILURE',
  SET_ERROR: 'SET_ERROR'
};

// Reducer function
const authReducer = (state, action) => {
  switch (action.type) {
    case actionTypes.LOGIN_START:
      return {
        ...state,
        loading: true,
        error: null
      };
    
    case actionTypes.LOGIN_SUCCESS:
      return {
        ...state,
        user: action.payload.user,
        token: action.payload.token,
        isAuthenticated: true,
        loading: false
      };
    
    case actionTypes.LOGIN_FAILURE:
      return {
        ...state,
        user: null,
        token: null,
        isAuthenticated: false,
        loading: false,
        error: action.payload
      };
    
    case actionTypes.LOGOUT:
      return {
        ...initialState,
        loading: false
      };
    
    case actionTypes.CHECK_AUTH_START:
      return {
        ...state,
        loading: true
      };
    
    case actionTypes.CHECK_AUTH_SUCCESS:
      return {
        ...state,
        user: action.payload.user,
        token: action.payload.token,
        isAuthenticated: true,
        loading: false
      };
    
    case actionTypes.CHECK_AUTH_FAILURE:
      return {
        ...initialState,
        loading: false
      };
    
    case actionTypes.SET_ERROR:
      return {
        ...state,
        error: action.payload,
        loading: false
      };
    
    default:
      return state;
  }
};

// Provider component
export const AuthProvider = ({ children }) => {
  const [state, dispatch] = useReducer(authReducer, initialState);
  const history = useHistory();
  const location = useLocation();

  // Check authentication status on app load
  useEffect(() => {
    checkAuthStatus();
  }, []);

  // Function to check authentication status
  const checkAuthStatus = async () => {
    dispatch({ type: actionTypes.CHECK_AUTH_START });
    
    try {
      // Check if token exists in localStorage
      const token = localStorage.getItem('auth_token');
      if (token) {
        // Verify token with backend (in a real implementation)
        // const response = await fetch('/api/auth/verify', {
        //   headers: { 'Authorization': `Bearer ${token}` }
        // });
        // 
        // if (response.ok) {
        //   const userData = await response.json();
        //   dispatch({ 
        //     type: actionTypes.CHECK_AUTH_SUCCESS, 
        //     payload: { user: userData, token } 
        //   });
        // } else {
        //   throw new Error('Token verification failed');
        // }
        
        // For this example, we'll just retrieve user data from localStorage
        const user = localStorage.getItem('user');
        if (user) {
          dispatch({ 
            type: actionTypes.CHECK_AUTH_SUCCESS, 
            payload: { user: JSON.parse(user), token } 
          });
        } else {
          throw new Error('No user data found');
        }
      } else {
        dispatch({ type: actionTypes.CHECK_AUTH_FAILURE });
      }
    } catch (error) {
      console.error('Auth check failed:', error);
      dispatch({ type: actionTypes.CHECK_AUTH_FAILURE });
    }
  };

  // Function to login user
  const login = async (email, password) => {
    dispatch({ type: actionTypes.LOGIN_START });
    
    try {
      // Call login API
      const response = await fetch('/api/auth/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password })
      });
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Login failed');
      }
      
      const data = await response.json();
      
      // Save token and user data to localStorage
      localStorage.setItem('auth_token', data.access_token);
      localStorage.setItem('user', JSON.stringify(data.user));
      
      dispatch({ 
        type: actionTypes.LOGIN_SUCCESS, 
        payload: { user: data.user, token: data.access_token } 
      });
      
      // Redirect to previous location or home
      const redirectTo = localStorage.getItem('redirectTo') || '/';
      localStorage.removeItem('redirectTo');
      history.push(redirectTo);
      
      return { success: true };
    } catch (error) {
      console.error('Login error:', error);
      dispatch({ type: actionTypes.LOGIN_FAILURE, payload: error.message });
      return { success: false, error: error.message };
    }
  };

  // Function to register user
  const register = async (userData) => {
    dispatch({ type: actionTypes.LOGIN_START });
    
    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(userData)
      });
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Registration failed');
      }
      
      const data = await response.json();
      
      // Save token and user data to localStorage
      // Note: In a real implementation, you'd need to log in after registration
      // or the API would return the token directly
      localStorage.setItem('user', JSON.stringify(data));
      
      dispatch({ 
        type: actionTypes.LOGIN_SUCCESS, 
        payload: { user: data, token: null } // Token would come from login after verification
      });
      
      return { success: true };
    } catch (error) {
      console.error('Registration error:', error);
      dispatch({ type: actionTypes.LOGIN_FAILURE, payload: error.message });
      return { success: false, error: error.message };
    }
  };

  // Function to logout user
  const logout = () => {
    // Clear from localStorage
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user');
    
    dispatch({ type: actionTypes.LOGOUT });
    
    // Redirect to home
    history.push('/');
  };

  const value = {
    ...state,
    login,
    register,
    logout,
    checkAuthStatus
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

// Custom hook to use the auth context
export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};

export default AuthContext;