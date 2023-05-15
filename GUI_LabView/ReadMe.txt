Due to that the GUI needs data streamed from the hardware to demonstrate the features, it 
might seem unfunctional now. To resolve this, I have enabled the user to change all the values
 manually on the front panel as if they were being received from the ESP32. Try to increase the 
“state machine” variable from -1 to 2 first (state 2 means ready to fly). Then, you can play with 
the yaw (0 to 360), pitch (-90 to 90), and roll (-180 to 180) values to examine the functionality of 
the attitude indicator. Next, try to increase “LeftPWM” or “RightPWM” from 3277 to 6544 
and observe the blades speeding up. Finally, you can try change the “command pitch” and 
“command yaw” between -30 and 30 and see the pink dot in the box on the right move with 
the input coordinates. (You will probably see some “hidden” variables as you scroll up the front 
panel. But, don’t worry, you won’t be able to interact with them since they are greyed out, and 
once the VI runs, they’ll disappear on the front panel completely. So, the user won’t be able to 
destroy this functional Vi!!!)