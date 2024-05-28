#!/bin/bash

# Function to create symlinks recursively
create_symlinks() {
  local src_dir=$1
  local dest_dir=$2

  # Ensure the source directory exists
  if [ ! -d "$src_dir" ]; then
    echo "Source directory $src_dir does not exist."
    return
  fi

  # Ensure the destination directory exists, create if not
  if [ ! -d "$dest_dir" ]; then
    mkdir -p "$dest_dir"
    echo "Created destination directory $dest_dir"
  fi

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

# Check for correct number of arguments
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <source_directory> <destination_directory>"
  exit 1
fi

# Start the recursive process from the provided source and destination directories
create_symlinks "$1" "$2"

echo "All symlinks and directories created."
