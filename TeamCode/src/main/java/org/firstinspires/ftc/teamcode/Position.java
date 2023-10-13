package org.firstinspires.ftc.teamcode;

/** Represents the position of the robot.
 *  More generally, it contains an x-y point and a rotation.
 */

 /**What is protected?
  - specifies that the member can only be accessed within its own package
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

     /** creates a point with the coordinates of (0,0), no name, a rotation of 0. and powersettings of 1
      *
      */
    public Position() { //This sets all the presets on the field as a blank or a 0.
       x = 0.0; //Sets X as a value of 0 on the field (where it starts)
       y = 0.0; //Sets y as a value of 0 on the field (where it starts)
       name = ""; //Name of the position (set as blank, used for debugging)
       setRotation(0.0); //Rotation is set as a value of 0 on the field
    }

     /**Creates a point at a defined position and rotation with a name
      * @param x the x coordinate of the position
      * @param y the y coordinate of the position
      * @param theta the direction the robot should face at theis posotion
      * @param name the name of this positon  (for debugging)
      */
    public Position(double x, double y, double theta, String name) {
        this.x = x; //Calls X from the Protected X (in this code/class)
        this.y = y; //Calls Y from the Protected Y (in this code/class)
        this.name = name; //Calls the String Name (in this code/class)
        setRotation(theta); //Set's the rotation as the unknown angle
    }

     /**Creates a point that has a definited position, rotation, name, strafePower, and rotatePower
      * @param x the x coordinate of the position
      * @param y the y coordinate of the position
      * @param theta the direction the robot should face at theis posotion
      * @param name the name of this positon  (for debugging)
      * @param strafePower the amount of power on the wheels when the robots strafe (speeds ranges from 0 to 1)
      * @param rotatePower the amount of power on the wheels when the robots spins (speeds ranges from 0 to 1)
      * @param action the action that should be perfomed when the robot reaches this position
      */
    public Position(double x, double y, String name, Navigation.Action action, double strafePower, double rotatePower, double theta) {
        this.x = x;
        this.y = y;
        this.name = name;
        setRotation(theta);
        this.strafePower = strafePower; //Sets strafePower from Line 23
        this.rotatePower = rotatePower; //Sets rotatePower form Line 24
        this.action = action;

    }

    //Sets all of the variables as a public function.
    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y){
        this.y = y;
    }
    public double getX() {
      return x;
    }
    public double getY(){
      return y;
    }
    public double getStrafePower(){
        return strafePower;
    }
    public double getRotatePower(){
        return rotatePower;
    }
    public double getRotation(){
        return rotation;
    }
    public Navigation.Action getAction(){
        return action;
    }
    public String getName(){
        return name;
    }

    /**
    why are you resetting a Point? that is not the Point of this class. THIS LITERALLY MAKES NO SENSE (lines 98 - 102
    */
    public void reset(){
        x=0; //setX(0.0); + setY(0.0);
        y=0;
        setRotation(0);
    }

    public static Position add(Position a, Position b) {
       return new Position(a.getX() + b.getX(), a.getY() + b.getY(), (a.getRotation() + b.getRotation()) % (2 * Math.PI),"");
    }
    /**sets the rotation of the position
    *p@param r the angle to set to
    *@return the instance of this class. (useful for chaining)
    */
    public Position setRotation(double r) {
      this.rotation = r;
      return this;
    }
}
//Programmers for position.java: Tyler M, Aria B, Manan C.
//Alumni Help: Stephen D.
