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
      <div className="container">
        <div className="row justify-content-center">
          <div className="col-md-6">
            <div className="card">
              <div className="card-body text-center">
                <h3 className="card-title">Account Created Successfully!</h3>
                <p>Thank you for signing up. Your technical background information has been saved.</p>
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
        <div className="col-md-8">
          <div className="card">
            <div className="card-header">
              <h3 className="card-title">Sign Up for Physical AI & Humanoid Robotics Textbook</h3>
              <p className="text-muted">Please provide your technical background to personalize your learning experience</p>
            </div>
            <div className="card-body">
              {error && (
                <div className="alert alert-danger" role="alert">
                  {error}
                </div>
              )}

              <form onSubmit={handleSubmit}>
                <div className="row">
                  <div className="col-md-6">
                    <div className="mb-3">
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
                  </div>
                  <div className="col-md-6">
                    <div className="mb-3">
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
                </div>

                <div className="mb-3">
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

                <div className="mb-4">
                  <h5>Technical Skills</h5>
                  <p className="text-muted">Select the technologies you have experience with:</p>
                  <div className="row">
                    {['ROS 2', 'NVIDIA Isaac', 'Gazebo', 'Unity', 'Python', 'C++', 'Machine Learning', 'Computer Vision', 'Reinforcement Learning', 'Path Planning', 'Manipulation', 'SLAM'].map(skill => (
                      <div className="col-md-4 mb-2" key={skill}>
                        <div className="form-check">
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
                      </div>
                    ))}
                  </div>
                </div>

                <div className="row mb-4">
                  <div className="col-md-6">
                    <div className="mb-3">
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

                <div className="mb-4">
                  <h5>Background Questionnaire</h5>
                  <div className="mb-3">
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

                  <div className="mb-3">
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

                  <div className="mb-3">
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

                <div className="d-grid">
                  <button type="submit" className="btn btn-primary btn-lg">Create Account</button>
                </div>
              </form>

              <div className="text-center mt-3">
                <p>Already have an account? <a href="/login">Log in here</a></p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default UserRegistration;