import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';

const UserRegistration = () => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    name: '',
    technical_skills: [],
    experience_level: 'beginner',
    background_questionnaire: {
      software_exp: '',
      hardware_access: '',
      ros_knowledge: ''
    }
  });
  const [error, setError] = useState('');
  const [success, setSuccess] = useState(false);
  const { signup } = useAuth();

  const handleChange = (e) => {
    const { name, value } = e.target;
    if (name.startsWith('background_')) {
      setFormData({
        ...formData,
        background_questionnaire: {
          ...formData.background_questionnaire,
          [name.replace('background_', '')]: value
        }
      });
    } else {
      setFormData({
        ...formData,
        [name]: value
      });
    }
  };

  const handleSkillChange = (e) => {
    const { value, checked } = e.target;
    let updatedSkills = [...formData.technical_skills];

    if (checked) {
      updatedSkills.push(value);
    } else {
      updatedSkills = updatedSkills.filter(skill => skill !== value);
    }

    setFormData({
      ...formData,
      technical_skills: updatedSkills
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    try {
      await signup(formData);
      setSuccess(true);
      setError('');
      // Optionally redirect to home or dashboard
    } catch (err) {
      setError(err.message || 'Signup failed. Please try again.');
      console.error('Signup error:', err);
    }
  };

  if (success) {
    return (
      <div className="auth-form-container">
        <div className="text--center">
          <h2>Account Created Successfully!</h2>
          <p>Thank you for signing up. Your technical background information has been saved.</p>
          <a href="/" className="button button--primary button--lg">Go to Home</a>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-form-container">
      <div className="auth-form-step-indicator">
        <h2>Sign Up for Physical AI & Humanoid Robotics Textbook</h2>
        <p>Please provide your technical background to personalize your learning experience</p>
      </div>

      {error && (
        <div className="alert alert--danger margin-bottom--md" role="alert">
          {error}
        </div>
      )}

      <form onSubmit={handleSubmit}>
        <div className="row">
          <div className="col col--6 margin-bottom--md">
            <label htmlFor="name" className="form-label">Full Name</label>
            <input
              type="text"
              className="form-control"
              id="name"
              name="name"
              value={formData.name}
              onChange={handleChange}
              required
            />
          </div>
          <div className="col col--6 margin-bottom--md">
            <label htmlFor="email" className="form-label">Email</label>
            <input
              type="email"
              className="form-control"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              required
            />
          </div>
        </div>

        <div className="form-group margin-bottom--md">
          <label htmlFor="password" className="form-label">Password</label>
          <input
            type="password"
            className="form-control"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            required
          />
        </div>

        <div className="margin-bottom--lg">
          <h3>Technical Skills</h3>
          <p className="text--secondary">Select the technologies you have experience with:</p>
          <div className="row">
            {['ROS 2', 'NVIDIA Isaac', 'Gazebo', 'Unity', 'Python', 'C++', 'Machine Learning', 'Computer Vision', 'Reinforcement Learning', 'Path Planning', 'Manipulation', 'SLAM'].map(skill => (
              <div className="col col--4" key={skill}>
                <label className="checkbox margin-bottom--sm">
                  <input
                    type="checkbox"
                    value={skill}
                    checked={formData.technical_skills.includes(skill)}
                    onChange={handleSkillChange}
                  />
                  <span className="checkbox__label">{skill}</span>
                </label>
              </div>
            ))}
          </div>
        </div>

        <div className="row margin-bottom--lg">
          <div className="col col--6">
            <div className="form-group">
              <label htmlFor="experience_level" className="form-label">Overall Experience Level</label>
              <select
                className="form-select"
                id="experience_level"
                name="experience_level"
                value={formData.experience_level}
                onChange={handleChange}
              >
                <option value="beginner">Beginner (0-1 years)</option>
                <option value="intermediate">Intermediate (2-4 years)</option>
                <option value="advanced">Advanced (5+ years)</option>
              </select>
            </div>
          </div>
        </div>

        <div className="margin-bottom--lg">
          <h3>Background Questionnaire</h3>
          <div className="form-group margin-bottom--md">
            <label htmlFor="background_software_exp" className="form-label">Software Development Experience</label>
            <textarea
              className="form-control"
              id="background_software_exp"
              name="background_software_exp"
              value={formData.background_questionnaire.software_exp}
              onChange={handleChange}
              placeholder="Describe your experience with software development, programming languages, and development tools..."
              rows="3"
            />
          </div>

          <div className="form-group margin-bottom--md">
            <label htmlFor="background_hardware_access" className="form-label">Hardware Access</label>
            <textarea
              className="form-control"
              id="background_hardware_access"
              name="background_hardware_access"
              value={formData.background_questionnaire.hardware_access}
              onChange={handleChange}
              placeholder="What hardware do you have access to? (simulation environments, physical robots, sensors, etc.)"
              rows="3"
            />
          </div>

          <div className="form-group margin-bottom--md">
            <label htmlFor="background_ros_knowledge" className="form-label">ROS Knowledge</label>
            <textarea
              className="form-control"
              id="background_ros_knowledge"
              name="background_ros_knowledge"
              value={formData.background_questionnaire.ros_knowledge}
              onChange={handleChange}
              placeholder="Describe your experience with ROS/ROS2, if any..."
              rows="3"
            />
          </div>
        </div>

        <div className="margin-top--lg">
          <button type="submit" className="button button--primary button--lg button--block">Create Account</button>
        </div>
      </form>

      <div className="text--center margin-top--lg">
        <p>Already have an account? <a href="/login">Log in here</a></p>
      </div>
    </div>
  );
};

export default UserRegistration;