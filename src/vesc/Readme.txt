//////////////////////////////////////////////////
/////////////// HOW TO DEBBUG ////////////////////
//////////////////////////////////////////////////

Failed to connect to the VESC, SerialException Failed to open the serial port to the VESC. IO Exception (13): Permission denied, file /home/lab-robot/Bureau/astek_ws/src/serial/src/impl/unix.cc, line 151. failed..

sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1

If you try to echo sensor/core and you get this message error :
	cannot load message class for .. Are your message built
then just source the terminal used to echo sensor core 
	source devel/setup.bash 





//////////////////////////////////////////////////
/////////////// HOW TO USE IT ////////////////////
//////////////////////////////////////////////////

If you want to command the left wheel speed in RPM by sending with a frequency of 80 HZ
	 rostopic pub -r 80 /left_wheel/commands/motor/speed std_msgs/Float64 "data: 5000" 


If you want to command to vehicule with twist ( linear and angular velocity) then enter this command .
For example to go straight forward at 0.8 m/s and publishing   at 80 Hz on the topic  /cmd_vel_to_rpm

    rostopic pub -r 80  /cmd_vel_to_rpm geometry_msgs/Twist -- '[0.8, 0.0, 0.0]' '[0.0, 0.0, 0.0]'



If you want to know the speed measured by the encoder on the right motor :
    rostopic echo /right_wheel/sensors/core

