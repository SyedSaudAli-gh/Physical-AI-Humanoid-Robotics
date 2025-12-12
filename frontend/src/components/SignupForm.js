import React, { useState } from 'react';
import { useAuth } from '../contexts/AuthContext';

const SignupForm = () => {
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
  const [currentStep, setCurrentStep] = useState(1); // Multi-step form
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
      // In Docusaurus, we can't use history.push, so we'll just show a success message
      // A real implementation might use window.location or some other navigation method
    } catch (err) {
      setError('Signup failed. Please try again.');
      console.error('Signup error:', err);
    }
  };

  if (success) {
    return (
      <div className="auth-form-container">
        <div className="text-center">
          <h2>Account Created Successfully!</h2>
          <p>Thank you for signing up. Please check your email to verify your account.</p>
          <a href="/" className="button button--primary">Go to Home</a>
        </div>
      </div>
    );
  }

  return (
    <div className="auth-form-container">
      <div className="auth-form-step-indicator">
        <h2>Create Account</h2>
        <p>Step {currentStep} of 2</p>
      </div>

      {currentStep === 1 && (
        <form onSubmit={(e) => { e.preventDefault(); setCurrentStep(2); }}>
          <div className="form-group margin-bottom--md">
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

          <div className="form-group margin-bottom--md">
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

          <div className="margin-top--lg">
            <button type="submit" className="button button--primary button--lg button--block">
              Continue to Background Info
            </button>
          </div>

          {error && (
            <div className="alert alert--danger margin-top--md" role="alert">
              {error}
            </div>
          )}
        </form>
      )}

      {currentStep === 2 && (
        <form onSubmit={handleSubmit}>
          <div className="margin-bottom--lg">
            <h3>Technical Background</h3>
            <p className="text--secondary">Help us personalize your learning experience</p>
          </div>

          <div className="margin-bottom--md">
            <p><strong>Technical Skills:</strong> Select all that apply</p>
            <div className="checkbox-group">
              {['ROS', 'NVIDIA Isaac', 'Gazebo', 'Unity', 'Python', 'C++', 'Machine Learning', 'Computer Vision'].map(skill => (
                <div className="form-group" key={skill}>
                  <label className="checkbox">
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

          <div className="form-group margin-bottom--md">
            <label htmlFor="experience_level" className="form-label">Experience Level</label>
            <select
              className="form-select"
              id="experience_level"
              name="experience_level"
              value={formData.experience_level}
              onChange={handleChange}
            >
              <option value="beginner">Beginner</option>
              <option value="intermediate">Intermediate</option>
              <option value="advanced">Advanced</option>
            </select>
          </div>

          <div className="form-group margin-bottom--md">
            <label htmlFor="background_software_exp" className="form-label">Software Experience</label>
            <textarea
              className="form-control"
              id="background_software_exp"
              name="background_software_exp"
              value={formData.background_questionnaire.software_exp}
              onChange={handleChange}
              placeholder="Describe your software development experience..."
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
              placeholder="What hardware do you have access to? (simulation environment, physical robots, etc.)"
              rows="3"
            />
          </div>

          <div className="form-group margin-bottom--lg">
            <label htmlFor="background_ros_knowledge" className="form-label">ROS Knowledge</label>
            <textarea
              className="form-control"
              id="background_ros_knowledge"
              name="background_ros_knowledge"
              value={formData.background_questionnaire.ros_knowledge}
              onChange={handleChange}
              placeholder="Describe your experience with ROS/ROS2..."
              rows="3"
            />
          </div>

          <div className="d-flex justify-content-between">
            <button
              type="button"
              className="button button--secondary"
              onClick={() => setCurrentStep(1)}
            >
              Back
            </button>
            <button type="submit" className="button button--primary">Create Account</button>
          </div>

          {error && (
            <div className="alert alert--danger margin-top--md" role="alert">
              {error}
            </div>
          )}
        </form>
      )}

      {currentStep === 1 && (
        <div className="text--center margin-top--lg">
          <p>Already have an account? <a href="/login">Login here</a></p>
        </div>
      )}
    </div>
  );
};

export default SignupForm;