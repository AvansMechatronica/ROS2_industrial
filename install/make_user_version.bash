#!/bin/bash


# Script to replace all occurrences of a given user path with the current user in all files
# Usage: ./make_user_version.bash <old_user>
# Example: ./make_user_version.bash student


set -e


# Check arguments
if [ $# -ne 1 ]; then
    echo "Error: Missing argument"
    echo "Usage: $0 <old_user>"
    echo "Example: $0 student"
    exit 1
fi

# Arguments
OLD_USER="$1"
NEW_USER="$(whoami)"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"


echo "Detected current user: $NEW_USER"
echo "Starting replacement of /$OLD_USER/ with /$NEW_USER/ in all files..."
echo "Working directory: $WORKSPACE_ROOT"

# Find all files (excluding .git directory and binary files)
# and replace old user path with new user path
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
