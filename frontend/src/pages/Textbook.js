import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Component to fetch and display textbook modules
const TextbookDisplay = () => {
  const [modules, setModules] = useState([]);
  const [selectedModule, setSelectedModule] = useState(null);
  const [chapters, setChapters] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  // Fetch all modules when component mounts
  useEffect(() => {
    const fetchModules = async () => {
      try {
        const response = await fetch('/api/modules'); // This would proxy to your backend
        if (!response.ok) {
          throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();
        setModules(data);
        setLoading(false);
      } catch (err) {
        setError(err.message);
        setLoading(false);
      }
    };

    fetchModules();
  }, []);

  // Fetch chapters when a module is selected
  useEffect(() => {
    if (selectedModule) {
      const fetchChapters = async () => {
        try {
          const response = await fetch(`/api/modules/${selectedModule.id}/chapters`);
          if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
          }
          const data = await response.json();
          setChapters(data);
        } catch (err) {
          setError(err.message);
        }
      };

      fetchChapters();
    } else {
      setChapters([]);
    }
  }, [selectedModule]);

  if (loading) {
    return (
      <Layout title="Loading...">
        <div className="container margin-vert--xl">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <h1>Loading textbook content...</h1>
              <p>Please wait while we load the modules.</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  if (error) {
    return (
      <Layout title="Error">
        <div className="container margin-vert--xl">
          <div className="row">
            <div className="col col--8 col--offset-2">
              <h1>Error loading textbook content</h1>
              <p>{error}</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Physical AI & Humanoid Robotics Book">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Physical AI & Humanoid Robotics Book</h1>
            <p>Explore the 4 core modules of the book</p>

            <div className="margin-vert--lg">
              <h2>Modules</h2>
              <div className="row">
                {modules.map((module) => (
                  <div key={module.id} className="col col--3 margin-vert--md">
                    <button
                      className={`button button--${selectedModule?.id === module.id ? 'primary' : 'secondary'}`}
                      onClick={() => setSelectedModule(module)}
                      style={{ width: '100%' }}
                    >
                      {module.name}
                    </button>
                  </div>
                ))}
              </div>
            </div>

            {selectedModule && (
              <div className="margin-vert--lg">
                <h3>{selectedModule.name}</h3>
                <p>{selectedModule.description}</p>

                <h4>Chapters</h4>
                <div className="card">
                  <div className="card__body">
                    {chapters.map((chapter) => (
                      <div key={chapter.id} className="margin-bottom--md">
                        <h5>{chapter.title}</h5>
                        <h6>Learning Outcomes:</h6>
                        <ul>
                          {chapter.learning_outcomes.map((outcome, idx) => (
                            <li key={idx}>{outcome}</li>
                          ))}
                        </ul>
                        <p><strong>Preview:</strong> {chapter.content_preview}</p>
                      </div>
                    ))}
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
};

export default TextbookDisplay;