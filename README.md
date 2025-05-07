# Elite Robot EI65-G71 6 joint Robot controller

Elite  Robotics (starting out as Beijing Elite Technology company in 2016) began making industrial robots like the EI65, but around 2017, they switched over to the Cobot collaborative robot trend.  However this little guy with a date code of 2018 seemed to be about the last of the efforts to make regular robot arms before they completely shifted to Cobots. As of 2025, almost all information pertaining to this robot is absent.  If you have one, I would REALLY like to know. 


The easiest way to run this python script is just load it up in VS Code and hit GO. It uses Python V3.11, and a smattering of other modules, but its all pretty straight forward.

How it works:

THe guys at Elite didnt stray too much from this initial go at robotics. Most of the current manuals ( as of 2025) has similar screenshots of the pendant and all the passwords were the same.  Since the controller used a Linux flavor, we could see that it had 2 important files for us: mcserver and robotmon.
-mcserver is presumably the motion controller, accepting about 30 commands, and among them is joint movement commands, stop commands and in some modes you can change the speed. Not all commands work and some are just downright confusing. Decompiling the binary is a slog, but its slowly providing clues on how its supposed to work
-robotmon is the robot joint and PLC input monitor. It leverages the display buffer and only updates the changing characters instead of entire strings. This was a fun surprise that needed special handling. I am guessing it was done to save on bandwidth.

The python is pretty straightforward. It slurps up the output of robotmon and teases out the joint positions into an actionable format. User input boxes allow for joint angle inputs, and those angles can be sent to the robot. The action buttons available are MOVE, STOP and SYNC. Sync will update the state of the robot and the input boxes to match the current joint angles of the robot.

Other interesteing bits:
-6DOF robot superstructure looks largely licensed from Mitsubishi. The Elite EI65 external dimensions are nearly identical to the Mitsubishi RV-7FR series.
-The robot arms servos, however are made by HCSV, various
-Servo drivers are HCSV HCSERVO-X3, nice!
-The pendant looks like a variant of NEWker robot controller. A lot of their graphics seem to match up.
-The robot control box has some sort of ARMv7 Processor rev 10 (Freescale i.MX6 Quad/DualLite) board with an Altera Cyclone-IV FPGA on it.
-Its running Linux version 3.10.18-rt15-ga183926 
