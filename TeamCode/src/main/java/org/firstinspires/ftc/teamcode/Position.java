package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a rotation.
 */

 /**What is protected?
  - specifies that the member can only be accesed within its own package
   (as with package-private) and, in addiction,
    by a subclass of its class in another package.
  */

public class Position {
    //Using protected branches to access both branches PUBLIC or PRIVATE
    //USE DOUBLES TO STORE FRACTIONAL NUMBERS IF BEING WRITTEN, OTHERWISE, IT CAN ONLY STORE AT WHOLE NUMBERS
     private double rotation; //Stores fractional numbers into a rotation class

     protected String name;
     protected double x; //Stores the Y value in the robot's position on the field
     protected double y; //Stores the Y value in the robot's position on the field
     protected Navigation.Action action = Navigation.Action.NONE;
     //Add constructors if needed
     protected double strafePower = 1.0; //Strafe is set to FULL POWER. Number can range ONLY from -1 to 1
     protected double rotatePower = 1.0; //Rotation is set to FULL POWER.

     Position() { //This sets all the presets on the field as a blank or a 0.
        x = 0.0; //Sets X as a value of 0 on the field (where it starts)
        y = 0.0; //Sets y as a value of 0 on the field (where it starts)
        name = ""; //Name of the position (set as blank, used for debugging)
        setRotation(0.0); //Rotation is set as a value of 0 on the field

     }

    Position(double x, double y, double theta, String name) {
        this.x = x; //Calls X from the Protected X (in this code/class)
        this.y = y; //Calls Y from the Protected Y (in this code/class)
        this.name = name; //Calls the String Name (in this code/class)
        setRotation(theta); //
    }

    Position(double x, String name, Navigation.Action action, double strafePower, double strafePower, double roatePower, double theta) {

    }


}
