#!/bin/bash

# Script to replace all occurrences of /gerard/ with /student/ in all files
# This creates a student version of the repository

set -e

# Constants
OLD_USER="gerard"
NEW_USER="student"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Starting replacement of /$OLD_USER/ with /$NEW_USER/ in all files..."
echo "Working directory: $WORKSPACE_ROOT"

# Find all files (excluding .git directory and binary files)
# and replace /gerard/ with /student/
find "$WORKSPACE_ROOT" -type f \
    -not -path "*/\.git/*" \
    -not -path "*/build/*" \
    -not -path "*/install/*" \
    -not -path "*/log/*" \
    -exec grep -l "/$OLD_USER/" {} \; 2>/dev/null | while read -r file; do
    
    # Skip binary files
    if file "$file" | grep -q "text"; then
        echo "Processing: $file"
        sed -i "s|/$OLD_USER/|/$NEW_USER/|g" "$file"
    fi
done

echo "Replacement complete!"
echo "All occurrences of /$OLD_USER/ have been replaced with /$NEW_USER/"
