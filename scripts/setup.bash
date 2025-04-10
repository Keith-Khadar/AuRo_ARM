source /opt/ros/jazzy/setup.bash

alias srcbrc="source ~/.bashrc"
alias home="cd $WORKSPACE_DIR"

# Setup colcon_cd
source "/usr/share/colcon_cd/function/colcon_cd.sh"
export _colcon_cd_root=$WORKSPACE_DIR
alias ccd="colcon_cd"

# Setup up colbuild
alias cb="colbuild"

# This will build the repository from wherever you are and take you back into the mil2 repo
colbuild() {
	local prev_dir
	prev_dir=$(pwd)        # Store the current directory
	cd $WORKSPACE_DIR || return # Change to your workspace
	if [ $# -eq 0 ]; then
		colcon build --symlink-install # Build the workspace
	else
		colcon build --symlink-install --packages-select "$@" # Build the workspace
	fi
	source ./install/setup.bash # Source the install script
	cd "$prev_dir" || return    # Return to the original directory
}

# Autocomplete for colbuild based on ROS 2 packages
_colbuild_autocomplete() {
	local cur
	cur="${COMP_WORDS[COMP_CWORD]}" # Get the current word being typed
	local packages

	# Fetch the list of packages from the ROS 2 workspace (replace this with your workspace)
	packages=$(cd $WORKSPACE_DIR && colcon list --names-only)

	mapfile -t package_array <<<"$packages"

	mapfile -t replacement <<<"$(compgen -W "${package_array[*]}" -- "$cur")"

	if [ ${#replacement[@]} -eq 0 ] || [ -z "$cur" ]; then
		COMPREPLY=("${package_array[@]}")
	else
		# Filter packages based on the current word (autocomplete logic)
		COMPREPLY=("${replacement[0]}")
	fi

}

# Bind the autocomplete function to the colbuild command
complete -F _colbuild_autocomplete colbuild
complete -F _colbuild_autocomplete cb
complete -F _colbuild_autocomplete colcon_cd
complete -F _colbuild_autocomplete ccd
