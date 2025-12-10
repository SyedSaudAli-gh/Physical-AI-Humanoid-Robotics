// API service using centralized configuration
import config from './config';

// API service functions using the centralized configuration
export const fetchModules = async () => {
  try {
    const apiUrl = config.getApiBaseUrl();
    const response = await fetch(`${apiUrl}/api/modules`);
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    return await response.json();
  } catch (error) {
    console.error('Error fetching modules:', error);
    throw error;
  }
};

export const fetchModuleChapters = async (moduleId) => {
  try {
    const apiUrl = config.getApiBaseUrl();
    const response = await fetch(`${apiUrl}/api/modules/${moduleId}/chapters`);
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    return await response.json();
  } catch (error) {
    console.error('Error fetching chapters:', error);
    throw error;
  }
};

export const fetchChapterContent = async (chapterId, difficulty = null, language = null) => {
  try {
    const apiUrl = config.getApiBaseUrl();
    let url = `${apiUrl}/api/chapters/${chapterId}`;
    const params = new URLSearchParams();
    if (difficulty) params.append('difficulty', difficulty);
    if (language) params.append('language', language);

    if (params.toString()) {
      url += `?${params.toString()}`;
    }

    const response = await fetch(url);
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    return await response.json();
  } catch (error) {
    console.error('Error fetching chapter content:', error);
    throw error;
  }
};

// Function to get the API base URL
export const getApiBaseUrl = () => {
  return config.getApiBaseUrl();
};