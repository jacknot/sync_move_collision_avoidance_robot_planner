#!/bin/bash
# Procedure to upgrade MoveIt PlanningSceneInterface object
# THE SCRIPT REQUIRE SUDO PRIVILEDGES
# DO NOT CHANGE THE FOLDER OF THE SCRIPT
#
# Authors: Jacopo Mora, Matteo Salvalai

MOVEIT_INTERFACE_DIR="/opt/ros/kinetic/lib/python2.7/dist-packages/moveit_commander"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SRC_DIR="$SCRIPT_DIR/src"
echo "Upgrade of the moveit python interface"
echo "Remember: sudo is needed, do NOT change the content of the folder"
echo "src contains the .py files that will be substituting the older versions in the installation dir"

#echo "DEBUG:Fix dir: $SCRIPT_DIR"
#echo "DEBUG:Src dir: $SRC_DIR"
echo "Upgrading:"
for PY in $(ls $SRC_DIR)
do
    if [ "${PY: -3}" == ".py" ]
    then
        echo "  - $PY"
        #echo "DEBUG:     $MOVEIT_INTERFACE_DIR/$PY"
        chmod +x "$SRC_DIR/$PY"
        echo "      - Substituting old version ..."
        rm -f "$MOVEIT_INTERFACE_DIR/$PY"
        #echo "DEBUG:     $SRC_DIR/$PY ->$MOVEIT_INTERFACE_DIR/$PY"
        cp "$SRC_DIR/$PY" "$MOVEIT_INTERFACE_DIR/$PY"
        echo "      - Compiling to .pyc ..."
        python -m compileall "$MOVEIT_INTERFACE_DIR/$PY"
    fi
done
