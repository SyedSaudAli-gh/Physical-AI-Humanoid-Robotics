import apiService from '../../../frontend/src/services/api';

// Mock fetch API
global.fetch = jest.fn();

describe('apiService', () => {
  beforeEach(() => {
    fetch.mockClear();
  });

  describe('sendQuery', () => {
    test('successfully sends a query and receives a response', async () => {
      const mockResponse = {
        response: 'This is the AI response',
        sources: [
          { title: 'Source 1', url: 'http://example.com/1', snippet: 'Some snippet' }
        ]
      };
      
      fetch.mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse
      });

      const result = await apiService.sendQuery('Test query');
      
      expect(fetch).toHaveBeenCalledWith('/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: 'Test query',
          selected_text: null,
        }),
      });
      
      expect(result).toEqual(mockResponse);
    });

    test('handles API errors correctly', async () => {
      fetch.mockResolvedValueOnce({
        ok: false,
        status: 500
      });

      await expect(apiService.sendQuery('Test query')).rejects.toThrow('HTTP error! status: 500');
    });

    test('includes selected text in the request when provided', async () => {
      const mockResponse = {
        response: 'This is the AI response',
        sources: []
      };
      
      fetch.mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse
      });

      await apiService.sendQuery('Test query', 'Selected text context');
      
      expect(fetch).toHaveBeenCalledWith('/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: 'Test query',
          selected_text: 'Selected text context',
        }),
      });
    });
  });
});