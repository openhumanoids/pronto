#!/bin/sh
prefix=$1

count=0
for f in *.png; do
    mv "$f" "out/$prefix$count".png
    count=`expr $count + 1`
done

# avconv -f image2 -i %d.png -r 12  foo.avi
