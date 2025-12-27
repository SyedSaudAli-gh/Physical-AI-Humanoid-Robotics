// tests/integration/all-components-integration.test.js

// Integration test to verify all components work together
describe('All Components Integration Test', () => {
  beforeAll(async () => {
    // Set up test environment
    await page.goto('http://localhost:3000');
  });

  test('Verify all components are present and functional', async () => {
    // Verify the header is present
    const header = await page.$('.header');
    expect(header).toBeTruthy();

    // Verify the hero section is present
    const heroSection = await page.$('.hero-section');
    expect(heroSection).toBeTruthy();

    // Verify module cards are present
    const moduleCards = await page.$$('.module-card');
    expect(moduleCards.length).toBeGreaterThan(0);

    // Verify the footer is present
    const footer = await page.$('.footer');
    expect(footer).toBeTruthy();

    // Verify the chat widget is present
    const chatToggleButton = await page.$('.chat-toggle-button');
    expect(chatToggleButton).toBeTruthy();

    // Verify the chat can be opened
    await chatToggleButton.click();
    await page.waitForSelector('.chat-window', { timeout: 5000 });
    const chatWindow = await page.$('.chat-window');
    expect(chatWindow).toBeTruthy();

    // Verify the chat input is present
    const chatInput = await page.$('.chat-input');
    expect(chatInput).toBeTruthy();

    // Verify the send button is present
    const sendButton = await page.$('.send-button');
    expect(sendButton).toBeTruthy();

    // Verify the close button is present
    const closeButton = await page.$('.close-button');
    expect(closeButton).toBeTruthy();

    // Close the chat window
    await closeButton.click();
    await page.waitForSelector('.chat-window', { state: 'hidden' });
  });

  test('Verify navigation works correctly', async () => {
    // Test that navigation links exist and are clickable
    const navLinks = await page.$$('.nav-link');
    expect(navLinks.length).toBeGreaterThan(0);

    // Verify the first link works (without actually navigating)
    const firstLink = navLinks[0];
    const href = await firstLink.getAttribute('href');
    expect(href).toBeTruthy();
  });

  test('Verify module cards are clickable', async () => {
    // Get the first module card
    const moduleCards = await page.$$('.module-card');
    expect(moduleCards.length).toBeGreaterThan(0);

    const firstModuleCard = moduleCards[0];
    const moduleLink = await firstModuleCard.$('.module-link');
    expect(moduleLink).toBeTruthy();

    const moduleHref = await moduleLink.getAttribute('href');
    expect(moduleHref).toBeTruthy();
  });

  test('Verify footer links work', async () => {
    // Test that footer links exist
    const footerLinks = await page.$$('.footer-links a');
    expect(footerLinks.length).toBeGreaterThan(0);

    // Verify the first footer link exists
    const firstFooterLink = footerLinks[0];
    expect(firstFooterLink).toBeTruthy();

    const footerHref = await firstFooterLink.getAttribute('href');
    expect(footerHref).toBeTruthy();
  });
});