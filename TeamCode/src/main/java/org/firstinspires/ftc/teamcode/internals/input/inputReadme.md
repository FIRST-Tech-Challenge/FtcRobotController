This is the folder for classes that handle input.

Each class interprets gamepad input differently.  For example, if we have a servo
that has been programmed with two positions, there are (at least) two ways we can
get the servo to toggle between those positions:

1. we could use two different buttons, each corresponding to a position
2. we could create a function that uses the same button to toggle between the positions

To be able to control a device how we want, we need to create a function that
interprets gamepad input in a certain way.  For the first option, we would need to
program the servo to go to its 0 position when "o" is pressed and its 1 position when
"x" is pressed.  For the second option, we would want the servo to switch positions when
"x" is pressed - This requires different functions.

The purpose of this folder is to put files with these functions.  Each function should
have its own file and be named "input."  The default gamepad input is gamepad1.x, but
this can be changed when the function is called in an OpMode/Script.