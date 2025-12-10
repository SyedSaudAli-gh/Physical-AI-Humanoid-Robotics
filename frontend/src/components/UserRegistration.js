import React, { useState } from 'react';
import Layout from '@theme/Layout';

// Component for user registration with technical skills collection
const UserRegistration = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    technicalSkills: [],
    experienceLevel: '',
    backgroundQuestionnaire: {
      years_experience: '',
      primary_language: '',
      educational_background: '',
      hardware_access: false
    }
  });
  
  const [selectedSkills, setSelectedSkills] = useState([]);
  const [errors, setErrors] = useState({});
  const [isLoading, setIsLoading] = useState(false);

  // Available technical skills for selection
  const technicalSkillsOptions = [
    'ROS 2', 'Unity', 'Python', 'NVIDIA Isaac', 'Gazebo', 'C++', 
    'Computer Vision', 'Machine Learning', 'Deep Learning', 'Reinforcement Learning',
    'Motion Planning', 'Control Systems', 'SLAM', 'Path Planning'
  ];

  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;
    
    if (name.startsWith('background_')) {
      const field = name.replace('background_', '');
      setFormData({
        ...formData,
        backgroundQuestionnaire: {
          ...formData.backgroundQuestionnaire,
          [field]: type === 'checkbox' ? checked : value
        }
      });
    } else {
      setFormData({
        ...formData,
        [name]: type === 'checkbox' ? checked : value
      });
    }
  };

  const handleSkillToggle = (skill) => {
    setSelectedSkills(prev => 
      prev.includes(skill) 
        ? prev.filter(s => s !== skill) 
        : [...prev, skill]
    );
  };

  const validateForm = () => {
    const newErrors = {};
    
    if (!formData.email) newErrors.email = 'Email is required';
    else if (!/\S+@\S+\.\S+/.test(formData.email)) newErrors.email = 'Email is invalid';
    
    if (!formData.password) newErrors.password = 'Password is required';
    else if (formData.password.length < 8) newErrors.password = 'Password must be at least 8 characters';
    
    if (!formData.name) newErrors.name = 'Name is required';
    
    if (selectedSkills.length === 0) newErrors.technicalSkills = 'Please select at least one technical skill';
    
    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    
    if (!validateForm()) return;
    
    setIsLoading(true);
    
    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          ...formData,
          technical_skills: selectedSkills
        })
      });
      
      const data = await response.json();
      
      if (response.ok) {
        alert('Registration successful! Please check your email to verify your account.');
        // Redirect to login or home page
      } else {
        alert(`Registration failed: ${data.detail || 'Unknown error'}`);
      }
    } catch (error) {
      console.error('Registration error:', error);
      alert('An error occurred during registration. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="Register">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>Register for Physical AI Textbook</h2>
                <p>Please provide your background information to personalize your learning experience</p>
              </div>
              
              <div className="card__body">
                <form onSubmit={handleSubmit}>
                  <div className="margin-bottom--md">
                    <label htmlFor="name">Full Name</label>
                    <input
                      type="text"
                      id="name"
                      name="name"
                      value={formData.name}
                      onChange={handleChange}
                      className={`form-control ${errors.name ? 'is-invalid' : ''}`}
                      placeholder="Enter your full name"
                    />
                    {errors.name && <div className="alert alert--danger">{errors.name}</div>}
                  </div>
                  
                  <div className="margin-bottom--md">
                    <label htmlFor="email">Email</label>
                    <input
                      type="email"
                      id="email"
                      name="email"
                      value={formData.email}
                      onChange={handleChange}
                      className={`form-control ${errors.email ? 'is-invalid' : ''}`}
                      placeholder="Enter your email"
                    />
                    {errors.email && <div className="alert alert--danger">{errors.email}</div>}
                  </div>
                  
                  <div className="margin-bottom--md">
                    <label htmlFor="password">Password</label>
                    <input
                      type="password"
                      id="password"
                      name="password"
                      value={formData.password}
                      onChange={handleChange}
                      className={`form-control ${errors.password ? 'is-invalid' : ''}`}
                      placeholder="Create a password"
                    />
                    {errors.password && <div className="alert alert--danger">{errors.password}</div>}
                  </div>
                  
                  <div className="margin-bottom--lg">
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
                          /> {level.charAt(0).toUpperCase() + level.slice(1)}
                        </label>
                      ))}
                    </div>
                  </div>
                  
                  <div className="margin-bottom--lg">
                    <label>Technical Skills (Select all that apply)</label>
                    <div className="form-group">
                      {technicalSkillsOptions.map(skill => (
                        <label key={skill} className="checkbox-inline margin-right--md">
                          <input
                            type="checkbox"
                            checked={selectedSkills.includes(skill)}
                            onChange={() => handleSkillToggle(skill)}
                          /> {skill}
                        </label>
                      ))}
                    </div>
                    {errors.technicalSkills && <div className="alert alert--danger">{errors.technicalSkills}</div>}
                  </div>
                  
                  <div className="margin-bottom--md">
                    <h4>Background Questionnaire</h4>
                    <div className="margin-bottom--sm">
                      <label htmlFor="years_experience">Years of Experience</label>
                      <input
                        type="number"
                        id="years_experience"
                        name="background_years_experience"
                        value={formData.backgroundQuestionnaire.years_experience}
                        onChange={handleChange}
                        className="form-control"
                        placeholder="e.g., 2"
                      />
                    </div>
                    
                    <div className="margin-bottom--sm">
                      <label htmlFor="primary_language">Primary Programming Language</label>
                      <input
                        type="text"
                        id="primary_language"
                        name="background_primary_language"
                        value={formData.backgroundQuestionnaire.primary_language}
                        onChange={handleChange}
                        className="form-control"
                        placeholder="e.g., Python, C++, etc."
                      />
                    </div>
                    
                    <div className="margin-bottom--sm">
                      <label htmlFor="educational_background">Educational Background</label>
                      <input
                        type="text"
                        id="educational_background"
                        name="background_educational_background"
                        value={formData.backgroundQuestionnaire.educational_background}
                        onChange={handleChange}
                        className="form-control"
                        placeholder="e.g., Computer Science, Robotics Engineering, etc."
                      />
                    </div>
                    
                    <div className="margin-bottom--sm">
                      <label className="checkbox-inline">
                        <input
                          type="checkbox"
                          name="background_hardware_access"
                          checked={formData.backgroundQuestionnaire.hardware_access}
                          onChange={handleChange}
                        /> Do you have access to hardware for practical exercises?
                      </label>
                    </div>
                  </div>
                  
                  <div className="margin-top--lg">
                    <button 
                      type="submit" 
                      className="button button--primary button--lg"
                      disabled={isLoading}
                    >
                      {isLoading ? 'Registering...' : 'Register'}
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

export default UserRegistration;