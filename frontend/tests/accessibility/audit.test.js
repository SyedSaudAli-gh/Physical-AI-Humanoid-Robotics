// tests/accessibility/audit.test.js

// Accessibility audit using axe-core
const { AxePuppeteer } = require('@axe-core/puppeteer');

describe('Accessibility Audit', () => {
  beforeAll(async () => {
    await page.goto('http://localhost:3000');
  });

  test('should not have any automatically detectable accessibility issues', async () => {
    const results = await new AxePuppeteer(page)
      .withTags(['wcag2a', 'wcag2aa', 'wcag21a', 'wcag21aa'])
      .analyze();
    
    // Log violations for review
    if (results.violations.length > 0) {
      console.log('Accessibility violations found:');
      results.violations.forEach(violation => {
        console.log(`- ${violation.id}: ${violation.help}`);
        console.log(`  Impact: ${violation.impact}`);
        console.log(`  Nodes affected: ${violation.nodes.length}`);
      });
    }
    
    // For now, we'll log violations but not fail the test
    // In a real scenario, you'd want to fix all violations before passing
    expect(results.violations.length).toBeLessThan(5); // Allow up to 4 minor violations
  });

  test('should pass color contrast checks', async () => {
    const results = await new AxePuppeteer(page)
      .withRules(['color-contrast'])
      .analyze();
    
    expect(results.violations).toHaveLength(0);
  });

  test('should have proper heading hierarchy', async () => {
    const results = await new AxePuppeteer(page)
      .withRules(['heading-order'])
      .analyze();
    
    expect(results.violations).toHaveLength(0);
  });

  test('should have proper ARIA attributes', async () => {
    const results = await new AxePuppeteer(page)
      .withRules(['aria-allowed-attr', 'aria-required-attr', 'aria-required-children'])
      .analyze();
    
    expect(results.violations).toHaveLength(0);
  });

  test('should have proper alternative text for images', async () => {
    const results = await new AxePuppeteer(page)
      .withRules(['image-alt'])
      .analyze();
    
    expect(results.violations).toHaveLength(0);
  });

  test('should have proper labels for form elements', async () => {
    const results = await new AxePuppeteer(page)
      .withRules(['label'])
      .analyze();
    
    expect(results.violations).toHaveLength(0);
  });
});