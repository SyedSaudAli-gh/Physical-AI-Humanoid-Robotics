/**
 * Performance monitoring utilities for tracking UI interactions
 */

/**
 * Measures the execution time of a function and logs it
 * @param {Function} fn - The function to measure
 * @param {string} name - Name of the operation for logging
 * @returns {Function} A wrapped function that includes performance tracking
 */
export const withPerformanceTracking = (fn, name) => {
  return async (...args) => {
    const start = performance.now();
    try {
      const result = await fn.apply(this, args);
      const end = performance.now();
      const duration = end - start;
      
      // Log performance metrics
      if (duration > 200) {
        console.warn(`Performance alert: ${name} took ${duration.toFixed(2)}ms (over 200ms threshold)`);
      } else {
        console.log(`${name} executed in ${duration.toFixed(2)}ms`);
      }
      
      return result;
    } catch (error) {
      const end = performance.now();
      const duration = end - start;
      console.error(`${name} failed after ${duration.toFixed(2)}ms:`, error);
      throw error;
    }
  };
};

/**
 * Measures and logs the time it takes for a component to render
 * @param {Function} renderFn - The rendering function to measure
 * @param {string} componentName - Name of the component for logging
 * @returns {Function} A wrapped function that includes performance tracking
 */
export const measureRenderTime = (renderFn, componentName) => {
  return (...args) => {
    const start = performance.now();
    const result = renderFn.apply(this, args);
    const end = performance.now();
    const duration = end - start;
    
    if (duration > 200) {
      console.warn(`Render performance alert: ${componentName} took ${duration.toFixed(2)}ms to render`);
    } else {
      console.log(`${componentName} rendered in ${duration.toFixed(2)}ms`);
    }
    
    return result;
  };
};

/**
 * Tracks the time between user action and response
 * @param {string} actionName - Name of the user action
 * @returns {Object} Object with start and end functions for tracking
 */
export const trackUserAction = (actionName) => {
  const start = performance.now();
  
  return {
    end: (additionalInfo = '') => {
      const end = performance.now();
      const duration = end - start;
      
      if (duration > 200) {
        console.warn(`User action performance alert: ${actionName} took ${duration.toFixed(2)}ms ${additionalInfo}`);
      } else {
        console.log(`${actionName} completed in ${duration.toFixed(2)}ms ${additionalInfo}`);
      }
    }
  };
};