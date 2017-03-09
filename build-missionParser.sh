#!/bin/bash

#get working directory
HOME=`pwd`

DSL_DIR=UNDERSEA_DSL

BUILD_DIR="$HOME/build"


#-------------------------------------------------------------------
#  Part 1: Create build directory if does not exist
#-------------------------------------------------------------------
if [ ! -d "$BUILD_DIR" ]; then
  mkdir build
fi


#-------------------------------------------------------------------
#  Part 2: Create mission parser
#-------------------------------------------------------------------
printf "\nCreating mission parser\n"


#go to initial directory & then to controller's directory
cd $DSL_DIR

mvn package

JAR="target/UNDERSEA_DSL-jar-with-dependencies.jar"

if [ -e "$JAR" ]; then
  cp $JAR $HOME/UNDERSEA_DSL.jar
fi

cd $HOME
