#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_utils"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mtb/sim_ws/bot_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mtb/sim_ws/bot_ws/install/lib/python2.7/dist-packages:/home/mtb/sim_ws/bot_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mtb/sim_ws/bot_ws/build" \
    "/usr/bin/python" \
    "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_utils/setup.py" \
    build --build-base "/home/mtb/sim_ws/bot_ws/build/fetch_pkg/bot_utils" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/mtb/sim_ws/bot_ws/install" --install-scripts="/home/mtb/sim_ws/bot_ws/install/bin"
