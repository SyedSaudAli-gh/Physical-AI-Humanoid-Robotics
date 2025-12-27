# Quickstart Guide: Project Cleanup - Remove Old Backend Folder

## Prerequisites

- Git installed and configured
- Appropriate permissions to modify project files
- Backup of the project (recommended before starting)

## Audit Phase

1. Search the project for any references to the "backend" folder:
```bash
grep -r "backend/" . --exclude-dir=.git
```

2. Check for imports or dependencies in configuration files:
```bash
grep -r "backend" . --include="*.py" --include="*.js" --include="*.json" --include="*.yml" --include="*.yaml" --exclude-dir=.git
```

3. Verify that rag-backend does not have any dependencies on the old backend:
```bash
cd rag-backend
grep -r "../backend" . || echo "No references to parent backend found"
```

## Cleanup Steps

1. Create a backup of the project before proceeding (optional but recommended):
```bash
cd ..
cp -r Physical-AI-Humanoid-Robotics Physical-AI-Humanoid-Robotics-backup
cd Physical-AI-Humanoid-Robotics
```

2. Remove the backend folder and all its contents:
```bash
rm -rf backend/
```

3. Clean up any remaining references to the old backend:
   - Edit any files that contained references to the backend folder
   - Update any configuration files that pointed to the old backend
   - Remove any environment variables pointing to the old backend

4. Verify the project still works:
   - Start the rag-backend service
   - Run the Docusaurus documentation site
   - Test all functionality to ensure nothing is broken

## Verification

After completing the cleanup:

1. Confirm the backend folder is removed:
```bash
ls -la | grep backend
```
This should return no results.

2. Verify rag-backend functionality:
```bash
cd rag-backend
python -m pytest tests/  # Run tests if available
```

3. Verify Docusaurus works:
```bash
cd frontend
npm run build
npm run serve
```

## Rollback Plan

If something goes wrong, you can restore from your backup:
```bash
cd ..
rm -rf Physical-AI-Humanoid-Robotics
mv Physical-AI-Humanoid-Robotics-backup Physical-AI-Humanoid-Robotics
```