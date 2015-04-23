find . -type d \( ! -name . \) -exec sh -c 'cd "{}" ; echo "Processing $PWD" ; ./../rosbag_filter.sh log.bag filtered.bag;' \;
