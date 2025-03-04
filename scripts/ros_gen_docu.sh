#!/bin/sh

NODE=$1

ros2 param list $NODE | grep -v -i constrain > /tmp/param_list

cat /tmp/param_list | while read l; do ros2 param describe $NODE $l; done \
 | sed -r 's/^ +//g' \
 | sed -r 's/^([^:]+):(.*)/\*\*\1\*\*:\2/' \
 | sed -r 's/^(.*)/> \1/g' \
 | sed -r 's/^(.+)$/\1\\/g' \
 | sed -r 's/^(> \*\*Para.*)/\n\1/'

