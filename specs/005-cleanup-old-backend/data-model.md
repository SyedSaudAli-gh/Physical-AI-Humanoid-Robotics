# Data Model: Project Cleanup - Remove Old Backend Folder

## Entities

### Old Backend Directory
- **Attributes**:
  - path: string (the directory path to be removed: /backend)
  - contents: array of strings (list of files and subdirectories in the backend folder)
  - size: integer (total size of the directory in bytes)
  - last_modified: Date (timestamp of the last modification)

- **Relationships**: 
  - May have references from other parts of the project
  - Contains obsolete code that is no longer used

- **Validation rules**:
  - path must exist in the project root
  - directory must contain files to be removed

### New Backend Directory (rag-backend)
- **Attributes**:
  - path: string (the directory path to be preserved: /rag-backend)
  - contents: array of strings (list of files and subdirectories in the rag-backend folder)
  - size: integer (total size of the directory in bytes)
  - last_modified: Date (timestamp of the last modification)

- **Relationships**:
  - Must have no dependencies on the old backend directory
  - Contains active code that must be preserved

- **Validation rules**:
  - path must exist in the project root
  - directory must be functional after cleanup

### Project References
- **Attributes**:
  - file_path: string (path to the file containing the reference)
  - line_numbers: array of integers (lines in the file where old backend is referenced)
  - reference_type: enum (import, path, configuration, environment variable)

- **Relationships**:
  - Points to files that may need updating during cleanup

- **Validation rules**:
  - All references to old backend must be identified and handled

### Cleanup Documentation
- **Attributes**:
  - removed_files: array of strings (list of all files and directories removed)
  - modified_files: array of strings (list of files that were modified to remove references)
  - timestamp: Date (when the cleanup was performed)
  - status: enum (completed, failed, partial)

- **Relationships**:
  - Records the changes made during the cleanup operation

- **Validation rules**:
  - Must contain a complete record of all changes made