#!/bin/bash
# Obtener el archivo de configuración de entorno
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/config.env"

# Permitir X11 forwarding
xhost +local:docker

# Verificar si el contenedor ya está en ejecución
if docker ps --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "El contenedor ya esta en ejecucion, abriendo un nuevo terminal"
    docker exec -it $CONTAINER_NAME bash -c "source /opt/ros/humble/setup.bash; exec bash"
else
    echo "El contenedor no esta en ejecucion, iniciandolo"

	CMD="export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/home/user/shared_volume/PX4-Autopilot &&\
	export CATKIN_WS=/home/user/shared_volume/Beacon-EIF-Localization-AR/ros2_ws && \
	export PX4_ROOT=/home/user/shared_volume && /bin/bash"

	docker run -it \
        --network $NETWORK \
        -e LOCAL_USER_ID="$(id -u)" \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        -v $X11_SOCKET:$X11_SOCKET:ro \
        -v /dev/dri:/dev/dri\
        --volume="$WORKSPACE_PATH:/home/user/shared_volume" \
        --volume="/dev/input:/dev/input" \
        --volume="$XAUTH:$XAUTH" \
        -env="XAUTHORITY=$XAUTH" \
        --publish 14556:14556/udp \
        --name=${CONTAINER_NAME} \
        --privileged \
        $IMAGE_NAME \
        bash -c "${CMD}"
fi