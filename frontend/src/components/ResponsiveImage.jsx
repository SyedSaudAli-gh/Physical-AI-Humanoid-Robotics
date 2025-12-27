/**
 * ResponsiveImage Component
 * Handles images with appropriate sizing for different screen densities
 */

import React from 'react';

const ResponsiveImage = ({ src, alt, className, fallbackSrc, ...props }) => {
  return (
    <img 
      src={src} 
      alt={alt} 
      className={className}
      srcSet={
        `${src} 1x, 
         ${src.replace(/\.\w+$/, (ext) => `@2x${ext}`)} 2x,
         ${src.replace(/\.\w+$/, (ext) => `@3x${ext}`)} 3x`
      }
      {...props}
      onError={(e) => {
        // Fallback to a default image if the original fails to load
        if (fallbackSrc) {
          e.target.src = fallbackSrc;
        }
      }}
    />
  );
};

export default ResponsiveImage;