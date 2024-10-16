########################
# To run HSLAM multiple instances you can use [&\] operator between two of the below commands
#Example:
#  ./HSLAM
#  &\
#  ./HSLAM
######################

#Set the desired directory path
build_directory_path="$HOME/hslam_ros_ws/src/HSLAM/build/bin/"
cd "$build_directory_path"


results_directory="$HOME/Dev/HSLAM//Results/"
mkdir -p $results_directory
repetitions=1
#hslam uncalib N times
 for ((i = 0; i < repetitions; i++)); do

    ./HSLAM \
        -files=/media/sf_datasets/ficosa_for_hslam/ficosa_may1/video0/images.zip \
        -calib=/media/sf_datasets/ficosa_for_hslam/camera_0.txt \
        -vocabPath=$HOME/Dev/HSLAM/misc/orbvoc.dbow3 \
        -LoopClosure=True \
        -preset=0 \
        -mode=2 \
            

    if [ -f "result.txt" ]; then
        # Move result.txt to the destination directory
        mv "result.txt" "$destination_directory/hslam_results_$i.txt"
        echo "Moved result.txt to: $destination_directory"

    fi
done