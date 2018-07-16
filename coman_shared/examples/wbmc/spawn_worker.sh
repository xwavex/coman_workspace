#! /bin/bash
killall worker
for i in {6..27}; do
  echo $i
  #gnome-terminal --command="./worker $i" &
  ./worker $i &
done
./worker 28
#./wbmc_main
