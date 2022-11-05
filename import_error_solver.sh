#!/bin/sh
pack_path=$(rospack find path_generator)
lib_path=$pack_path/../../devel/.private/path_generator/lib/path_generator
mkdir -p $lib_path
rm -rf $lib_path/*.py
ln $pack_path/scripts/*.py $lib_path
