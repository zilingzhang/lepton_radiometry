# lepton_radiometry
simple python ROS wrapper for flir lepton 3.5 radiometry, publish raw sensor data (16bit per pixel, resolution 0.01K, start from absolute zero) and gray scale image (grayscale range clamp by max and min temp in a frame)

# Allow un-privelege accesss to the thermal camera
sudo sh -c "echo 'SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"1e4e\", ATTRS{idProduct}==\"0100\", SYMLINK+=\"pt1\", GROUP=\"usb\", MODE=\"666\"' > /etc/udev/rules.d/99-pt1.rules"
