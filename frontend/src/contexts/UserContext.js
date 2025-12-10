// contexts/UserContext.js
import React, { createContext, useContext, useReducer, useEffect } from 'react';

// Create the context
const UserContext = createContext();

// Initial state
const initialState = {
  user: null,
  preferences: {},
  loading: true,
  error: null
};

// Reducer function
function userReducer(state, action) {
  switch (action.type) {
    case 'SET_LOADING':
      return {
        ...state,
        loading: action.payload
      };
    case 'SET_USER':
      return {
        ...state,
        user: action.payload,
        loading: false,
        error: null
      };
    case 'SET_PREFERENCES':
      return {
        ...state,
        preferences: action.payload,
        loading: false,
        error: null
      };
    case 'UPDATE_PREFERENCES':
      return {
        ...state,
        preferences: { ...state.preferences, ...action.payload },
        loading: false
      };
    case 'SET_ERROR':
      return {
        ...state,
        error: action.payload,
        loading: false
      };
    case 'LOGOUT':
      return {
        ...state,
        user: null,
        preferences: {},
        loading: false
      };
    default:
      return state;
  }
}

// Provider component
export const UserProvider = ({ children }) => {
  const [state, dispatch] = useReducer(userReducer, initialState);

  // Load user and preferences from localStorage or API on initial load
  useEffect(() => {
    const loadUserData = async () => {
      try {
        dispatch({ type: 'SET_LOADING', payload: true });
        
        // Simulate loading user data (in a real app, this would be an API call)
        const storedUser = localStorage.getItem('user');
        const storedPreferences = localStorage.getItem('preferences');
        
        if (storedUser) {
          dispatch({ type: 'SET_USER', payload: JSON.parse(storedUser) });
        }
        
        if (storedPreferences) {
          dispatch({ type: 'SET_PREFERENCES', payload: JSON.parse(storedPreferences) });
        } else {
          // Default preferences if none exist
          const defaultPreferences = {
            preferred_language: 'en',
            chapter_difficulty_override: null
          };
          localStorage.setItem('preferences', JSON.stringify(defaultPreferences));
          dispatch({ type: 'SET_PREFERENCES', payload: defaultPreferences });
        }
      } catch (error) {
        dispatch({ type: 'SET_ERROR', payload: error.message });
      } finally {
        dispatch({ type: 'SET_LOADING', payload: false });
      }
    };

    loadUserData();
  }, []);

  // Actions
  const login = async (userData) => {
    try {
      dispatch({ type: 'SET_USER', payload: userData });
      localStorage.setItem('user', JSON.stringify(userData));
    } catch (error) {
      dispatch({ type: 'SET_ERROR', payload: error.message });
    }
  };

  const logout = async () => {
    try {
      dispatch({ type: 'LOGOUT' });
      localStorage.removeItem('user');
      localStorage.removeItem('preferences');
      
      // Optionally redirect to home page or login page
    } catch (error) {
      dispatch({ type: 'SET_ERROR', payload: error.message });
    }
  };

  const updatePreferences = async (preferencesData) => {
    try {
      dispatch({ type: 'UPDATE_PREFERENCES', payload: preferencesData });
      localStorage.setItem('preferences', JSON.stringify({ ...state.preferences, ...preferencesData }));
    } catch (error) {
      dispatch({ type: 'SET_ERROR', payload: error.message });
    }
  };

  const value = {
    user: state.user,
    preferences: state.preferences,
    loading: state.loading,
    error: state.error,
    login,
    logout,
    updatePreferences
  };

  return (
    <UserContext.Provider value={value}>
      {children}
    </UserContext.Provider>
  );
};

// Custom hook to use the user context
export const useUser = () => {
  const context = useContext(UserContext);
  if (!context) {
    throw new Error('useUser must be used within a UserProvider');
  }
  return context;
};