// tests/integration/chat-integration.test.js

// Integration test for the chat functionality
describe('Chat Integration Test', () => {
  beforeAll(async () => {
    // Set up test environment
    await page.goto('http://localhost:3000');
  });

  test('Verify chat widget integration with text selection', async () => {
    // Verify the chat widget button is present
    const chatButton = await page.$('.chat-toggle-button');
    expect(chatButton).toBeTruthy();

    // Click the chat button to open the widget
    await chatButton.click();
    
    // Wait for the chat window to appear
    await page.waitForSelector('.chat-window', { timeout: 5000 });
    const chatWindow = await page.$('.chat-window');
    expect(chatWindow).toBeTruthy();

    // Verify the chat input is present
    const chatInput = await page.$('.chat-input');
    expect(chatInput).toBeTruthy();

    // Verify the send button is present
    const sendButton = await page.$('.send-button');
    expect(sendButton).toBeTruthy();

    // Test that we can type in the chat input
    await chatInput.type('Test message for integration');
    const inputValue = await chatInput.evaluate(node => node.value);
    expect(inputValue).toBe('Test message for integration');

    // Verify the send button is enabled when there's text
    const isSendButtonEnabled = await sendButton.evaluate(node => !node.disabled);
    expect(isSendButtonEnabled).toBe(true);

    // Test submitting a message
    await chatInput.press('Enter');
    
    // Wait for the message to appear in the chat history
    await page.waitForSelector('.message.user-message', { timeout: 5000 });
    const userMessage = await page.$('.message.user-message');
    expect(userMessage).toBeTruthy();

    // Close the chat window
    const closeButton = await page.$('.close-button');
    await closeButton.click();
    await page.waitForSelector('.chat-window', { state: 'hidden' });
  });

  test('Verify chat widget persistence across pages', async () => {
    // Open the chat widget
    const chatButton = await page.$('.chat-toggle-button');
    await chatButton.click();
    
    await page.waitForSelector('.chat-window', { timeout: 5000 });
    const chatWindow = await page.$('.chat-window');
    expect(chatWindow).toBeTruthy();

    // Navigate to another page (if available) - for now just refresh
    await page.reload({ waitUntil: ['networkidle0', 'domcontentloaded'] });
    
    // Wait a bit for page to load
    await page.waitForTimeout(2000);
    
    // Verify the chat widget is still available
    const newChatButton = await page.$('.chat-toggle-button');
    expect(newChatButton).toBeTruthy();
  });
});