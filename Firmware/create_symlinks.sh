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

# List of source and destination directory pairs, uncommet section of interest

# For CartPoleFirmware on STM (First create CartPoleFirmware project in Cube IDE!)
:'
declare -a directories=(
  "./Src/CartPoleFirmware ./CubeIDE/CartPoleFirmware/Src"
  "./Src/STM ./CubeIDE/CartPoleFirmware/Src/STM"
)
'

# For CartPoleFirmware on Zynq (First create CartPoleFirmware project in Vitis!)

declare -a directories=(
  "./Src/CartPoleFirmware ./VitisProjects/CartPoleFirmware/src"
  "./Src/Zynq ./VitisProjects/CartPoleFirmware/src/Zynq"
)


# For NeuralImitator on Zynq (First create NeuralImitator project in Vitis!)
:'
declare -a directories=(
  "./Src/NeuralImitator ./VitisProjects/NeuralImitator/src"
  "./Src/Zynq ./VitisProjects/NeuralImitator/src/Zynq"
)
'


##########################################################################################################
##########################################################################################################
##########################################################################################################

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

# Loop through the directory pairs and run the create_symlinks function for each pair
for dirs in "${directories[@]}"; do
  # Split the pair into source and destination
  src_dir=$(echo $dirs | awk '{print $1}')
  dest_dir=$(echo $dirs | awk '{print $2}')

  # Run the create_symlinks function with the current source and destination
  create_symlinks "$src_dir" "$dest_dir"
done

echo "All symlinks and directories created."
