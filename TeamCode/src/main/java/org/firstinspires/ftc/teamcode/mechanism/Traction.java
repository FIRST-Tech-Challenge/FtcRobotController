package org.firstinspires.ftc.teamcode.mechanism;


public class Traction {
    ProgrammingBoard Board = new ProgrammingBoard();

    // We have to go through "Board" to use the motors.

    public void controllerDrive(double axial, double lateral, double yaw){
        // Function takes in desired directions as parameters.

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial + lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial - lateral - yaw;
        // Calculates necessary motor speeds.

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        // Finds max value.


        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        // Readjusts for max value to preserve the intention of the controller.

        Board.setDCMotorPower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
        // Sends the desired inputs over to the Board class.






    }
}
