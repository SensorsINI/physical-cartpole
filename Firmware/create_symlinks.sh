#!/bin/bash
# This script create symbolic links in DEST_DIR from all files in SOURCE_DIR.
# To make the links work on other machines the links use relative paths.
# If there is a direcotry encountered inside SOURCE_DIR,
# the script will create such directory in DEST_DIR and populate it with symlinks
# to the respective files, applied recursively.
# In other words no symlinks to directories are created
# but the folders structure is recreated at destination directory
# and symlinks are created to all nested content 
# Also if DEST_DIR does not exist, it will be created.


# Define the source and destination directories
SOURCE_DIR="./Src/CartPoleFirmware"
DEST_DIR="./VitisProjects/CartPoleFirmware/src/"

# Ensure the source directory exists
if [ ! -d "$SOURCE_DIR" ]; then
  echo "Source directory $SOURCE_DIR does not exist."
  exit 1
fi

# Ensure the destination directory exists, create if not
if [ ! -d "$DEST_DIR" ]; then
  mkdir -p "$DEST_DIR"
  echo "Created destination directory $DEST_DIR"
fi

# Function to create symlinks recursively
create_symlinks() {
  local src_dir=$1
  local dest_dir=$2

  # Loop through all items in the source directory
  for item in "$src_dir"/*; 
  do
    # Extract the name from the path
    name=$(basename "$item")
    
    if [ -f "$item" ]; then
      # If it's a file, create the symlink
      ln -s "$(realpath --relative-to="$dest_dir" "$item")" "$dest_dir/$name"
      echo "Created symlink for $name in $dest_dir"
      
    elif [ -d "$item" ]; then
      # If it's a directory, create the directory in the destination and recurse
      mkdir -p "$dest_dir/$name"
      echo "Created directory $dest_dir/$name"
      create_symlinks "$item" "$dest_dir/$name"
    fi
  done
}

# Start the recursive process from the source and destination root directories
create_symlinks "$SOURCE_DIR" "$DEST_DIR"

echo "All symlinks and directories created."
