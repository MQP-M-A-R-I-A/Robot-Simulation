#####################################
##### Set up User and Group IDs #####
#####################################
USER_ID=$(id -u)
GROUP_ID=$(id -g)
BUILD_ENV=.devcontainer/build.env

rm $BUILD_ENV

printf "USER_ID=%s\n" "$USER_ID" >> $BUILD_ENV
printf "GROUP_ID=%s\n" "$GROUP_ID" >> $BUILD_ENV

##################################################
##### Set up x-server x-authentication files #####
##################################################
# XSOCK=.devcontainer/.X11-unix
# XAUTH=.devcontainer/.docker.xauth
# XSOCK=/tmp/.X11-unix
# XAUTH=/tmp/.docker.xauth

# printf "XSOCK=%s\n" "$XSOCK" >> .devcontainer/xauth.env
# printf "XAUTH=%s\n" "$XAUTH" >> .devcontainer/xauth.env

# rm $XAUTH && touch $XAUTH
# xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
# xhost +local:root