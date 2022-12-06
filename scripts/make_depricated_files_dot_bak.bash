function fail() {
    echo -e "\nFailed; exiting...\n"
    exit 1
}

function print_rename() {
  # Turns something like
  # "./java/org/firstinspires/ftc/teamcode/opmodes/deprecated/ftclibscheduler/DepositCommand.java"
  # into
  # "./java/../opmodes/deprecated/ftclibscheduler/DepositCommand.java"
  clean_path=$(echo "$1" | sed -E 's/org\/firstinspires\/ftc\/teamcode(kt)?/../')

  # Echoes something like
  # "Renaming './java/../opmodes/deprecated/ftclibscheduler/DepositCommand.java' to 'DepositCommand.java.bak'"
  echo "Renamed '$clean_path' to '$(basename "$1").bak'"
}

# Gets the path of the file the script is currently in
parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" || fail ; pwd -P )

# Changes directory to the TeamCode source files
# This is agnostic to the location of where the script was called;
# it will always run relative to the script's location
cd "$parent_path"/../TeamCode/src/main || fail

# Finds all the code files in 'TeamCode/src/main' that are in a 'deprecated' folder
# and renames them to have a '.bak' extension
while IFS= read -r -d '' dir; do
   while IFS= read -r -d '' file; do
      mv "$file" "$file.bak"
      print_rename "$file"
   done < <( find "$dir" \( -name '*.java' -or -name '*.kt' \) -print0 )
done < <( find . -type d -name "deprecated" -print0 )
