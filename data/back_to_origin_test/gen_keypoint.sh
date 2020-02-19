for i in {2..197}
do
	echo "Generating keypoint file $i"
	y1=$(printf "%03d" "$((i-1))")
	y2=$(printf "%03d" "$i")
	y3=$(printf "%03d" "$((i+1))")
	"../../cmake-build-debug/KeypointMatching" "img_$y1.jpg" "img_$y2.jpg" "img_$y3.jpg" "keypoint_$i.txt"
done