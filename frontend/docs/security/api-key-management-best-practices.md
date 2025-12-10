# Comprehensive Guide to API Key Management

This document provides detailed best practices for managing API keys and sensitive information in the Physical AI & Humanoid Robotics project and other software applications.

## 1. Understanding API Keys

API keys are unique identifiers used to authenticate and authorize access to services. They act as passwords that allow your application to communicate with external services like OpenAI, Cohere, Gemini, and databases.

## 2. Types of API Keys & Tokens

### Different Kinds of Keys
- **Service Account Keys**: Used for authentication with cloud services
- **Bearer Tokens**: Used for API authentication
- **Secret Keys**: Used for signing and verifying data
- **Database Connection Strings**: Contain credentials for database access

### Permissions and Scopes
Always use keys with minimal required permissions:
- Limit access to specific resources
- Apply time-based expiration where possible
- Use read-only keys when write access isn't required

## 3. Secure Storage

### ❌ Never Store Keys Like This:
```bash
# DON'T store keys directly in code
const OPENAI_API_KEY = "sk-actual-key-here"

# DON'T store keys in public repositories
export OPENAI_API_KEY="sk-actual-key-here"

# DON'T hardcode keys in your application
public static final String API_KEY = "sk-actual-key-here";
```

### ✅ Proper Storage Methods:
- Use environment variables for runtime configuration
- Store in dedicated secrets management systems (HashiCorp Vault, AWS Secrets Manager, Azure Key Vault)
- Utilize cloud provider's secret management solutions (AWS Systems Manager Parameter Store, Google Cloud Secret Manager)

## 4. Environment-Specific Keys

Use different API keys for different environments:
- Development: Use dedicated development keys with limited permissions
- Staging: Use staging keys that mirror production as closely as possible
- Production: Use full-privileged production keys

Example configuration structure:
```
.env.development
.env.staging
.env.production
```

## 5. Revocation and Rotation

### Regular Rotation Schedule
- Rotate keys every 3-6 months (or according to your security policy)
- Establish automated rotation for some services
- Maintain a rotation calendar

### Immediate Action Steps When Keys Are Compromised
1. Revoke the exposed key immediately
2. Generate a new key with the same permissions
3. Deploy the new key to all affected systems
4. Monitor for suspicious activity during the transition period

### Audit Trail
- Keep records of who had access to keys
- Document when keys were rotated
- Log key usage where possible

## 6. Application-Level Security

### Backend Proxy Pattern
As demonstrated in this project, never expose backend API keys to frontend code:

✅ **Recommended Architecture:**
1. Frontend makes requests to your backend API
2. Backend handles all external API calls using stored keys
3. Backend returns processed data to frontend

❌ **Avoid Direct Client Access:**
1. Never send API keys to the browser
2. Don't make direct requests from client to external APIs

### Implement Rate Limiting
- Protect your API keys from accidental overuse
- Implement request throttling
- Monitor API usage patterns for anomalies

## 7. Team Access Management

### Principle of Least Privilege
- Grant access only to team members who need it
- Use role-based access control (RBAC)
- Regularly audit team member access

### Onboarding & Offboarding
- Include API key access in onboarding checklist
- Remove access immediately during offboarding
- Use temporary access tokens when possible

## 8. Monitoring & Detection

### Logging
- Log API key usage (without logging the actual keys)
- Monitor for unusual access patterns
- Alert on failed authentication attempts

### Tools for Detection
Consider using:
- GitGuardian or similar tools to detect committed secrets
- GitHub's secret scanning
- Static analysis tools during CI/CD

## 9. Incident Response Plan

Have a documented response plan for when API keys are exposed:

1. **Immediate Response:**
   - Identify all systems that may have used the key
   - Revoke the exposed key
   - Generate new keys

2. **Communication:**
   - Notify relevant team members
   - Inform stakeholders if customer data could be affected
   - Document the incident

3. **Remediation:**
   - Update all affected systems with new keys
   - Conduct post-mortem to prevent future incidents
   - Review and update security policies as needed

## 10. Additional Security Measures

### Network Security
- Use HTTPS/TLS for all API communications
- Implement firewall rules where possible
- Consider VPN for highly sensitive environments

### Code Security
- Use linters to catch common mistakes
- Implement pre-commit hooks to scan for secrets
- Conduct security reviews of pull requests

### Compliance
- Adhere to applicable regulations (GDPR, HIPAA, etc.)
- Document data handling procedures
- Regular security audits

## 11. Specific Service Recommendations

### OpenAI API Keys
- Apply usage limits in your OpenAI dashboard
- Monitor token usage regularly
- Restrict domains that can make requests if applicable

### Cohere API Keys
- Monitor requests and usage patterns
- Use Cohere's team management features appropriately

### Qdrant Vector Database
- Use authentication tokens with minimal required permissions
- Consider IP whitelisting if possible

### Database Connections
- Use strong, randomly generated passwords
- Implement SSL encryption for connections
- Regularly rotate database credentials

## 12. Common Pitfalls to Avoid

1. **Accidental Commits**: Always use .gitignore for environment files
2. **Clipboard Accidents**: Clear clipboard after pasting keys
3. **Screenshot Sharing**: Never include keys in screenshots
4. **Plain Text Storage**: Encrypt sensitive files
5. **Shared Accounts**: Use individual accounts when possible
6. **Weak Passwords**: Use randomly generated keys with sufficient entropy

## 13. Resources for Further Learning

- OWASP API Security Top 10
- NIST Cybersecurity Framework
- Cloud Provider Security Documentation
- Your company's security guidelines