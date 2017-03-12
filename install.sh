#!/bin/bash
# clean mbed and cp bin

MOUNT=NODE_F303K8 
VOLUMEDIR=/Volumes 
SRC=./BUILD/controller.bin

TARGET=$VOLUMEDIR/$MOUNT

echo Cleaning $TARGET and putting $SRC

rm -rf $TARGET/*
cp $SRC $TARGET