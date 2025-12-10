import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useUser } from '../contexts/UserContext';

// User Profile Dashboard
const UserProfile = () => {
  const { user, preferences, updatePreferences, loading, logout } = useUser();
  const [editing, setEditing] = useState(false);
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    technicalSkills: [],
    experienceLevel: '',
    backgroundQuestionnaire: {
      years_experience: '',
      primary_language: '',
      educational_background: '',
      hardware_access: false
    },
    preferred_language: 'en',
    chapter_difficulty_override: null
  });
  const [saveSuccess, setSaveSuccess] = useState(false);
  const [activeTab, setActiveTab] = useState('profile');

  // Load user data when available
  useEffect(() => {
    if (user) {
      setFormData({
        name: user.name || '',
        email: user.email || '',
        technicalSkills: user.technical_skills || [],
        experienceLevel: user.experience_level || '',
        backgroundQuestionnaire: user.background_questionnaire || {
          years_experience: '',
          primary_language: '',
          educational_background: '',
          hardware_access: false
        },
        preferred_language: preferences?.preferred_language || 'en',
        chapter_difficulty_override: preferences?.chapter_difficulty_override || null
      });
    }
  }, [user, preferences]);

  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;
    
    if (name.startsWith('background_')) {
      const field = name.replace('background_', '');
      setFormData(prev => ({
        ...prev,
        backgroundQuestionnaire: {
          ...prev.backgroundQuestionnaire,
          [field]: type === 'checkbox' ? checked : value
        }
      }));
    } else if (name === 'preferred_language' || name === 'chapter_difficulty_override') {
      setFormData(prev => ({
        ...prev,
        [name]: value
      }));
    } else {
      setFormData(prev => ({
        ...prev,
        [name]: value
      }));
    }
  };

  const handleSkillToggle = (skill) => {
    setFormData(prev => ({
      ...prev,
      technicalSkills: prev.technicalSkills.includes(skill)
        ? prev.technicalSkills.filter(s => s !== skill)
        : [...prev.technicalSkills, skill]
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    try {
      setSaveSuccess(false);
      
      // Update preferences
      await updatePreferences({
        preferred_language: formData.preferred_language,
        chapter_difficulty_override: formData.chapter_difficulty_override
      });
      
      // TODO: In a full implementation, you would also update user profile details
      // This would require a new API endpoint to update user background information
      
      setSaveSuccess(true);
      setTimeout(() => setSaveSuccess(false), 3000);
    } catch (error) {
      console.error('Error updating profile:', error);
    }
  };

  const technicalSkillsOptions = [
    'ROS 2', 'Unity', 'Python', 'NVIDIA Isaac', 'Gazebo', 'C++', 
    'Computer Vision', 'Machine Learning', 'Deep Learning', 'Reinforcement Learning',
    'Motion Planning', 'Control Systems', 'SLAM', 'Path Planning'
  ];

  if (loading) {
    return (
      <Layout title="Loading Profile...">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className="text--center padding--xl">
                <div className="loading-spinner"></div>
                <p>Loading your profile...</p>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (!user) {
    return (
      <Layout title="Profile">
        <div className="container margin-vert--lg">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <div className="text--center padding--xl">
                <p>Please log in to view your profile</p>
              </div>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="User Profile">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <h1>User Profile</h1>
            <p>Manage your profile and learning preferences</p>
          </div>
        </div>
        
        <div className="row">
          {/* Sidebar Navigation */}
          <div className="col col--3">
            <div className="card">
              <div className="card__body">
                <ul className="navbar-sidebar">
                  <li>
                    <button 
                      className={`button ${activeTab === 'profile' ? 'button--primary' : 'button--outline'}`}
                      onClick={() => setActiveTab('profile')}
                    >
                      Profile Info
                    </button>
                  </li>
                  <li className="margin-top--sm">
                    <button 
                      className={`button ${activeTab === 'preferences' ? 'button--primary' : 'button--outline'}`}
                      onClick={() => setActiveTab('preferences')}
                    >
                      Learning Preferences
                    </button>
                  </li>
                  <li className="margin-top--sm">
                    <button 
                      className="button button--outline button--block"
                      onClick={logout}
                    >
                      Logout
                    </button>
                  </li>
                </ul>
              </div>
            </div>
          </div>
          
          {/* Main Content */}
          <div className="col col--9">
            <div className="card">
              <div className="card__header">
                <h2>
                  {activeTab === 'profile' ? 'Profile Information' : 'Learning Preferences'}
                </h2>
              </div>
              
              <div className="card__body">
                {saveSuccess && (
                  <div className="alert alert--success">
                    Profile updated successfully!
                  </div>
                )}
                
                <form onSubmit={handleSubmit}>
                  {activeTab === 'profile' ? (
                    <div>
                      <div className="margin-bottom--md">
                        <label htmlFor="name">Full Name</label>
                        <input
                          type="text"
                          id="name"
                          name="name"
                          value={formData.name}
                          onChange={handleChange}
                          className="form-control"
                          disabled={!editing}
                        />
                      </div>
                      
                      <div className="margin-bottom--md">
                        <label htmlFor="email">Email</label>
                        <input
                          type="email"
                          id="email"
                          name="email"
                          value={formData.email}
                          onChange={handleChange}
                          className="form-control"
                          disabled
                        />
                        <small className="text--muted">Email cannot be changed</small>
                      </div>
                      
                      <div className="margin-bottom--md">
                        <label>Experience Level</label>
                        <div className="form-group">
                          {['beginner', 'intermediate', 'advanced'].map(level => (
                            <label key={level} className="radio-inline margin-right--md">
                              <input
                                type="radio"
                                name="experienceLevel"
                                value={level}
                                checked={formData.experienceLevel === level}
                                onChange={handleChange}
                                disabled={!editing}
                              /> {level.charAt(0).toUpperCase() + level.slice(1)}
                            </label>
                          ))}
                        </div>
                      </div>
                      
                      <div className="margin-bottom--md">
                        <label>Technical Skills</label>
                        <div className="form-group">
                          {technicalSkillsOptions.map(skill => (
                            <label key={skill} className="checkbox-inline margin-right--md">
                              <input
                                type="checkbox"
                                checked={formData.technicalSkills.includes(skill)}
                                onChange={() => handleSkillToggle(skill)}
                                disabled={!editing}
                              /> {skill}
                            </label>
                          ))}
                        </div>
                      </div>
                      
                      <div className="margin-bottom--md">
                        <h4>Background Questionnaire</h4>
                        
                        <div className="row">
                          <div className="col col--6">
                            <label htmlFor="background_years_experience">Years of Experience</label>
                            <input
                              type="number"
                              id="background_years_experience"
                              name="background_years_experience"
                              value={formData.backgroundQuestionnaire.years_experience}
                              onChange={handleChange}
                              className="form-control"
                              disabled={!editing}
                            />
                          </div>
                          
                          <div className="col col--6">
                            <label htmlFor="background_primary_language">Primary Language</label>
                            <input
                              type="text"
                              id="background_primary_language"
                              name="background_primary_language"
                              value={formData.backgroundQuestionnaire.primary_language}
                              onChange={handleChange}
                              className="form-control"
                              disabled={!editing}
                            />
                          </div>
                        </div>
                        
                        <div className="margin-top--sm">
                          <label htmlFor="background_educational_background">Educational Background</label>
                          <input
                            type="text"
                            id="background_educational_background"
                            name="background_educational_background"
                            value={formData.backgroundQuestionnaire.educational_background}
                            onChange={handleChange}
                            className="form-control"
                            disabled={!editing}
                          />
                        </div>
                        
                        <div className="margin-top--sm">
                          <label className="checkbox-inline">
                            <input
                              type="checkbox"
                              name="background_hardware_access"
                              checked={formData.backgroundQuestionnaire.hardware_access}
                              onChange={handleChange}
                              disabled={!editing}
                            /> Access to hardware for practical exercises
                          </label>
                        </div>
                      </div>
                    </div>
                  ) : (
                    <div>
                      <div className="margin-bottom--md">
                        <label htmlFor="preferred_language">Preferred Language</label>
                        <select
                          id="preferred_language"
                          name="preferred_language"
                          value={formData.preferred_language}
                          onChange={handleChange}
                          className="form-control"
                        >
                          <option value="en">English</option>
                          <option value="ur">Urdu</option>
                        </select>
                      </div>
                      
                      <div className="margin-bottom--md">
                        <label htmlFor="chapter_difficulty_override">Default Chapter Difficulty</label>
                        <select
                          id="chapter_difficulty_override"
                          name="chapter_difficulty_override"
                          value={formData.chapter_difficulty_override || 'default'}
                          onChange={handleChange}
                          className="form-control"
                        >
                          <option value="default">Auto-select based on profile</option>
                          <option value="beginner">Beginner</option>
                          <option value="intermediate">Intermediate</option>
                          <option value="advanced">Advanced</option>
                        </select>
                        <small className="text--muted">
                          This will override the automatic difficulty selection based on your profile
                        </small>
                      </div>
                    </div>
                  )}
                  
                  <div className="margin-top--lg">
                    <button 
                      type="submit" 
                      className="button button--primary"
                      disabled={loading}
                    >
                      {loading ? 'Saving...' : 'Save Changes'}
                    </button>
                    
                    <button 
                      type="button" 
                      className="button button--outline margin-left--sm"
                      onClick={() => {
                        setEditing(!editing);
                        // Reset form to saved values when toggling edit mode
                        if (editing) {
                          setFormData({
                            name: user.name || '',
                            email: user.email || '',
                            technicalSkills: user.technical_skills || [],
                            experienceLevel: user.experience_level || '',
                            backgroundQuestionnaire: user.background_questionnaire || {
                              years_experience: '',
                              primary_language: '',
                              educational_background: '',
                              hardware_access: false
                            },
                            preferred_language: preferences?.preferred_language || 'en',
                            chapter_difficulty_override: preferences?.chapter_difficulty_override || null
                          });
                        }
                      }}
                    >
                      {editing ? 'Cancel' : 'Edit Profile'}
                    </button>
                  </div>
                </form>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default UserProfile;