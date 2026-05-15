# mini_serial_demo

This is a simple demo for controlling MINI serial communication, including
basic serial data transmission and reception. It currently uses ROS 1 to
publish the MINI odometry topic. ROS 2 is not supported yet.

## Communication Parameters

The UART connector is a GH1.0 5-pin interface. Starting from the screw-hole
side, the pin order from left to right is:

```text
GND, TRIG, TX, RX, 5V
```

- Baud rate: `115200`
- Logic level: `3.3 V`

## Build and Run

```
mkdir -p ~/mini_serial_demo
cd ~/mini_serial_demo
git clone https://github.com/Hessian-matrix/mini_serial_demo
catkin_make
source devel/setup.bash
roslaunch serial_demo serial_demo.launch
```

## Runtime Behavior

After launch, the terminal waits for keyboard input:

```text
[ INFO] [1770238508.251035632]: please input command: 1-start, 2-stop, 3-reset, 4-exit, a-switch algo
```

Available commands:

- `1`: Start the algorithm
- `2`: Stop the algorithm
- `3`: Reset the algorithm
- `4`: Exit the program
- `a`: Switch the algorithm
