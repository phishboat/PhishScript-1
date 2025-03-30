# PhishScript-1
## Client-server Arduino scripts for Phish, rotates one direction
### Server 
At three potentiometers, receives user inputs for frequency, power, and base speed.  
Connects to client  
Transmits UDP packets with user input
### Client  
Connects to server  
Outputs PWM signals in sine wave corresponding to user input
