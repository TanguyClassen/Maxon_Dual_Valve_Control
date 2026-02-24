# Maxon_Dual_Valve_Control
Python software to control a valve driven by a maxon motor running on EPOS4 via USB or rs232




=========== MAXON MOTOR TESTER =========

Please run the system as follows :

1.SETUP
	a. Hardware
		Power your EPOS card after connecting your Motor + Sensor bundle to the EPOS.
		Then plug the board via USB to your computer running the programm.
	
	b. Software setup
		Once your motor(s) have been connected via usb, choose the correct USB port (by default, the first physical usb port should be "USB1" but that could be different on your device)
			To find the port you will be using, run this full line script in your terminal :

				Get-CimInstance Win32_PnPEntity|Where-Object{$_.Name-match"COM"-and$_.PNPDeviceID-like"USB*"}|Select-Object @{Name="Device";Expression={$_.Name}},@{Name="COMPort";Expression={[regex]::Match($_.Name,"COM\d+").Value}},@{Name="USBLocation";Expression={$_.LocationInformation}},@{Name="PNPDeviceID";Expression={$_.PNPDeviceID}}
				
		Then enter the node ID of your board (if you are controlling two motors, make sure to connect them to different nodes to be sure this works properly...)
		Finally, enter your system parameters.


1.Calibration
	You won't be able to move the motors without calibrating them first (they need to know their allowed range of motion)
	To calibrate your valves, use the "open" and "close" buttons to move your motor to the desired positions, once at the desired closed or open position, click on ""
