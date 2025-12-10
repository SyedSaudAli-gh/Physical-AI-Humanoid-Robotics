import { createAuthClient } from '@better-auth/client';

// Initialize Better-Auth client
export const authClient = createAuthClient({
  baseURL: 'http://localhost:8000', // This should match your backend URL
  // For production, this would be the production backend URL
});

export default authClient;