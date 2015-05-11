find . -type d \( ! -name . \) -exec sh -c 'cd "{}" ; echo "Processing $PWD" ; rosbag play -i filtered.bag;' \;
