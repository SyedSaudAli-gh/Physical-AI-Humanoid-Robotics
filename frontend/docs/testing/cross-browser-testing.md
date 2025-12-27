# Cross-Browser Testing Guide

This document outlines the cross-browser testing strategy for the Physical AI & Humanoid Robotics textbook website.

## Supported Browsers

- Chrome (latest 2 versions)
- Firefox (latest 2 versions)
- Safari (latest 2 versions)
- Edge (latest 2 versions)
- Mobile Safari (iOS)
- Chrome Mobile (Android)

## Testing Checklist

### Core Functionality
- [ ] Homepage loads correctly
- [ ] Navigation works properly
- [ ] Chat widget opens and closes
- [ ] Text selection functionality works
- [ ] Chat messages send and receive properly
- [ ] Module cards are clickable and navigate correctly
- [ ] Footer links work properly
- [ ] Responsive design works on mobile and tablet

### Accessibility Features
- [ ] ARIA labels are properly implemented
- [ ] Keyboard navigation works correctly
- [ ] Screen reader compatibility
- [ ] Focus management works properly
- [ ] Color contrast meets WCAG standards

### Performance
- [ ] Initial load time < 200ms
- [ ] Page transitions are smooth
- [ ] Chat responses are timely
- [ ] Images load properly
- [ ] No memory leaks

## Testing Tools

- BrowserStack
- Sauce Labs
- CrossBrowserTesting
- Local virtual machines for older browser versions

## Known Issues

- If any browser-specific issues are found, document them here with workarounds
- Include browser version, OS, and steps to reproduce

## Automated Testing

### Setup
```bash
npm install --save-dev browserstack-local
```

### Configuration
Create a `wdio.conf.js` file with BrowserStack configurations for automated cross-browser testing.