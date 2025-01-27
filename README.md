SC2104 Sensor Interface & Design, Computer Engineering course taken in AY24S2.
For all the labs, 1-5, you will be using the STM32 board.
For lab 1, you will be introduced to the STM32 and learn how to set it up using the relevant applications to interface it.
For lab 2-3, you will be interfacing with the Intertial Measurement Unit (IMU) of the STM32. You will then implement a complimentary filter and a kalman filter to the gyroscope and accelerometer readings.
For lab 4, you will use the readings from the IMU to create a PID control system for a motor with a wire attached to it to control its reaction to stopping at a given angle
For lab 5, you will be using the readings from the IMU to stabilise a levelling platform with servos attached to it. You will use a PID controller to it so that it will remain level when you tilt and rotate it

Problems faced:
1. Accelerometer and gyroscope readings not coming out in the console
2. Readings come out on console but not in serial plot
3. Readings giving very large values

Solutions:
1. Plug out all the connections and wait for a while OR change the board
   - I personally prefer changing the boards as there are times when you can get readings but when you run the program again the issue occurs
2. Make sure that when printing out onto the console, your last character should not be a comma (,) else will not come out on serial plot
3. Make sure to declare your variables and set them to the correct values

Reccomendations:
- Try the labs yourself before looking at the mine or others code. Better to learn the hard way so that you will understand it better
- Go for free access before the labs. Although it is 3 hours, it is not sufficient. Try to complete the tasks before coming to lab. You can start going for pre labs for lab 2 onwards
