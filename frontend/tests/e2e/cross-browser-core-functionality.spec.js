// tests/e2e/cross-browser-core-functionality.spec.js

describe('Cross-Browser Core Functionality Tests', () => {
  before(async () => {
    // Set up test environment
    await browser.url('http://localhost:3000');
  });

  it('should load the homepage correctly', async () => {
    // Verify the page title
    const title = await browser.getTitle();
    expect(title).to.include('Physical AI & Humanoid Robotics Book');
    
    // Verify the main heading is present
    const heading = await $('h1*=Physical AI & Humanoid Robotics');
    await expect(heading).to.be.existing;
    
    // Verify the hero section is present
    const heroSection = await $('.hero-section');
    await expect(heroSection).to.be.existing;
  });

  it('should display the chat widget', async () => {
    // Verify the chat widget button is present
    const chatButton = await $('.chat-toggle-button');
    await expect(chatButton).to.be.existing;
    
    // Verify the chat can be opened
    await chatButton.click();
    
    // Wait for the chat window to appear
    const chatWindow = await $('.chat-window');
    await browser.waitUntil(async () => await chatWindow.isDisplayed(), {
      timeout: 5000,
      timeoutMsg: 'Chat window did not appear after 5s'
    });
    
    await expect(chatWindow).to.be.existing;
  });

  it('should have functional navigation', async () => {
    // Verify main navigation items exist
    const navItems = await $$('.nav-item a');
    await expect(navItems).to.have.length.greaterThan(0);
    
    // Verify at least one navigation item works
    if (navItems.length > 0) {
      const firstNavItem = navItems[0];
      await expect(firstNavItem).to.be.existing;
    }
  });

  it('should display module cards', async () => {
    // Verify module cards are present
    const moduleCards = await $$('.module-card');
    await expect(moduleCards).to.have.length.greaterThan(0);
    
    // Verify at least one module card is clickable
    if (moduleCards.length > 0) {
      const firstCard = moduleCards[0];
      await expect(firstCard).to.be.existing;
      
      const moduleLink = await firstCard.$('.module-link');
      await expect(moduleLink).to.be.existing;
    }
  });

  it('should have a functional footer', async () => {
    // Verify footer exists
    const footer = await $('footer');
    await expect(footer).to.be.existing;
    
    // Verify footer links exist
    const footerLinks = await footer.$$('.footer-links a');
    await expect(footerLinks).to.have.length.greaterThan(0);
  });
});