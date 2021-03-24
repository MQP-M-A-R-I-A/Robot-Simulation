export USER_ID=$(id -u)
export GROUP_ID=$(id -g)

rm .devcontainer/build.env

printf "USER_ID=%s\n" "$USER_ID" >> .devcontainer/build.env
printf "GROUP_ID=%s\n" "$GROUP_ID" >> .devcontainer/build.env