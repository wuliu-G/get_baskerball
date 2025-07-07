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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/r2/test_G/src/my_pkg1"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/r2/test_G/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/r2/test_G/install/lib/python3/dist-packages:/home/r2/test_G/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/r2/test_G/build" \
    "/usr/bin/python3" \
    "/home/r2/test_G/src/my_pkg1/setup.py" \
     \
    build --build-base "/home/r2/test_G/build/my_pkg1" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/r2/test_G/install" --install-scripts="/home/r2/test_G/install/bin"
