echo "Please select the camera you want to calibrate: (1. Camera 1, 2. Camera 2, 3. Camera 3))"

# Provide the user with options to choose from
options=("Camera 1" "Camera 2" "Camera 3")

# Use the select statement to display the options
select opt in "${options[@]}"
do
    case $opt in
        "Camera 1")
            # Run camera calibration command for Camera 1
            rosrun camera_calibration cameracalibrator.py --size 6x5 --square 0.025 --k-coefficients=2 --fix-principal-point i --fix-aspect-ratio image:=/camera_1/image_raw camera:=/camera_1
            break
            ;;
        "Camera 2")
            # Run camera calibration command for Camera 2
            rosrun camera_calibration cameracalibrator.py --size 6x5 --square 0.025 --k-coefficients=2 --fix-principal-point i --fix-aspect-ratio image:=/camera_2/image_raw camera:=/camera_2
            break
            ;;
        "Camera 3")
            # Run camera calibration command for Camera 3
            rosrun camera_calibration cameracalibrator.py --size 6x5 --square 0.025 --k-coefficients=2 --fix-principal-point i --fix-aspect-ratio image:=/camera_3/image_raw camera:=/camera_3
            break
            ;;
        *) echo "Invalid option selected";;
    esac
done