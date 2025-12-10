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
      <div className="container">
        <div className="row justify-content-center">
          <div className="col-md-6">
            <div className="card">
              <div className="card-body text-center">
                <h3 className="card-title">Account Created Successfully!</h3>
                <p>Thank you for signing up. Please check your email to verify your account.</p>
                <a href="/" className="btn btn-primary">Go to Home</a>
              </div>
            </div>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="container">
      <div className="row justify-content-center">
        <div className="col-md-6">
          <div className="card">
            <div className="card-header">
              <h3>Create Account</h3>
              <p>Step {currentStep} of 2</p>
            </div>
            <div className="card-body">
              {currentStep === 1 && (
                <form onSubmit={(e) => { e.preventDefault(); setCurrentStep(2); }}>
                  <div className="form-group mb-3">
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

                  <div className="form-group mb-3">
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

                  <div className="form-group mb-3">
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

                  <div className="d-grid">
                    <button type="submit" className="btn btn-primary">Next: Background Info</button>
                  </div>
                </form>
              )}

              {currentStep === 2 && (
                <form onSubmit={handleSubmit}>
                  <div className="mb-3">
                    <p><strong>Technical Skills:</strong></p>
                    {['ROS', 'NVIDIA Isaac', 'Gazebo', 'Unity', 'Python', 'C++', 'Machine Learning', 'Computer Vision'].map(skill => (
                      <div className="form-check" key={skill}>
                        <input
                          className="form-check-input"
                          type="checkbox"
                          id={`skill-${skill}`}
                          value={skill}
                          checked={formData.technical_skills.includes(skill)}
                          onChange={handleSkillChange}
                        />
                        <label className="form-check-label" htmlFor={`skill-${skill}`}>
                          {skill}
                        </label>
                      </div>
                    ))}
                  </div>

                  <div className="form-group mb-3">
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

                  <div className="form-group mb-3">
                    <label htmlFor="background_software_exp" className="form-label">Software Experience</label>
                    <textarea
                      className="form-control"
                      id="background_software_exp"
                      name="background_software_exp"
                      value={formData.background_questionnaire.software_exp}
                      onChange={handleChange}
                      placeholder="Describe your software development experience..."
                    />
                  </div>

                  <div className="form-group mb-3">
                    <label htmlFor="background_hardware_access" className="form-label">Hardware Access</label>
                    <textarea
                      className="form-control"
                      id="background_hardware_access"
                      name="background_hardware_access"
                      value={formData.background_questionnaire.hardware_access}
                      onChange={handleChange}
                      placeholder="What hardware do you have access to? (simulation environment, physical robots, etc.)"
                    />
                  </div>

                  <div className="form-group mb-3">
                    <label htmlFor="background_ros_knowledge" className="form-label">ROS Knowledge</label>
                    <textarea
                      className="form-control"
                      id="background_ros_knowledge"
                      name="background_ros_knowledge"
                      value={formData.background_questionnaire.ros_knowledge}
                      onChange={handleChange}
                      placeholder="Describe your experience with ROS/ROS2..."
                    />
                  </div>

                  <div className="d-flex justify-content-between">
                    <button
                      type="button"
                      className="btn btn-secondary"
                      onClick={() => setCurrentStep(1)}
                    >
                      Back
                    </button>
                    <button type="submit" className="btn btn-primary">Sign Up</button>
                  </div>
                </form>
              )}

              {error && (
                <div className="alert alert-danger mt-3" role="alert">
                  {error}
                </div>
              )}
            </div>
          </div>

          {currentStep === 1 && (
            <div className="text-center mt-3">
              <p>Already have an account? <a href="/login">Login here</a></p>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default SignupForm;