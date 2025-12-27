/**
 * Face Detection component for the Physical AI & Humanoid Robotics textbook
 * Uses face-api.js for facial recognition in the browser
 */

import React, { useEffect, useRef, useState } from 'react';
import './FaceDetection.css';

const FaceDetection = () => {
  const videoRef = useRef();
  const canvasRef = useRef();
  const [isCameraActive, setIsCameraActive] = useState(false);
  const [error, setError] = useState(null);
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    let intervalId;
    
    const loadModels = async () => {
      try {
        setLoading(true);
        // Import face-api.js
        const faceapi = await import('face-api.js');

        // Load the models
        await faceapi.nets.tinyFaceDetector.loadFromUri('/models');
        await faceapi.nets.faceLandmark68Net.loadFromUri('/models');
        await faceapi.nets.faceRecognitionNet.loadFromUri('/models');
        await faceapi.nets.faceExpressionNet.loadFromUri('/models');

        setLoading(false);
      } catch (err) {
        setError('Failed to load face detection models. Please make sure the models are available at /models');
        setLoading(false);
        console.error('Error loading face detection models:', err);
      }
    };

    // Load models when component mounts
    loadModels();

    return () => {
      if (intervalId) {
        clearInterval(intervalId);
      }
      stopCamera();
    };
  }, []);

  const startCamera = async () => {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ 
        video: { width: 640, height: 480 } 
      });
      
      const video = videoRef.current;
      video.srcObject = stream;
      video.play();
      
      setIsCameraActive(true);
      setError(null);
      
      // Start face detection
      startFaceDetection();
    } catch (err) {
      setError('Camera access denied or unavailable. Please check permissions.');
      console.error('Error accessing camera:', err);
    }
  };

  const stopCamera = () => {
    if (videoRef.current && videoRef.current.srcObject) {
      const tracks = videoRef.current.srcObject.getTracks();
      tracks.forEach(track => track.stop());
      setIsCameraActive(false);
    }
  };

  const startFaceDetection = async () => {
    const video = videoRef.current;
    const canvas = canvasRef.current;
    const faceapi = await import('face-api.js');

    const detect = async () => {
      if (video.readyState === 4) {
        // Set canvas dimensions to match video
        canvas.width = video.videoWidth;
        canvas.height = video.videoHeight;

        // Detect faces
        const detections = await faceapi.detectAllFaces(
          video,
          new faceapi.TinyFaceDetectorOptions()
        ).withFaceLandmarks().withFaceExpressions();

        // Draw detections on canvas
        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Draw face detections
        faceapi.draw.drawDetections(canvas, detections);
        faceapi.draw.drawFaceLandmarks(canvas, detections);
        faceapi.draw.drawFaceExpressions(canvas, detections);
      }

      // Continue detection if camera is still active
      if (isCameraActive) {
        requestAnimationFrame(detect);
      }
    };

    detect();
  };

  return (
    <div className="face-detection-container">
      <h3>Face Detection</h3>
      
      {loading && <div className="loading">Loading face detection models...</div>}
      
      {error && <div className="error">{error}</div>}
      
      <div className="camera-controls">
        {!isCameraActive ? (
          <button onClick={startCamera} className="start-camera-btn">
            Start Face Detection
          </button>
        ) : (
          <button onClick={stopCamera} className="stop-camera-btn">
            Stop Camera
          </button>
        )}
      </div>
      
      <div className="camera-container">
        <video 
          ref={videoRef} 
          className="video-element" 
          autoPlay 
          muted
          playsInline
        />
        <canvas ref={canvasRef} className="canvas-overlay" />
      </div>
      
      <div className="instructions">
        <p>Click "Start Face Detection" to begin. The system will detect faces in the camera feed.</p>
      </div>
    </div>
  );
};

export default FaceDetection;