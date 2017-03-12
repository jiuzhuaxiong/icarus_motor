#!/bin/bash
# clean mbed and cp bin

MOUNT=NODE_F303K8 
VOLUMEDIR=/Volumes 
SRC=./BUILD/controller.bin

TARGET=$VOLUMEDIR/$MOUNT

echo cp $SRC $TARGET
cp $SRC $TARGET/