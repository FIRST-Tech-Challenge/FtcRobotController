package org.firstinspires.ftc.teamcode.mechanism;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Traction {
    ProgrammingBoard Board = new ProgrammingBoard();

    // We have to go through "Board" to use the motors.

//    private DcMotor leftFrontMotor_0 = null;
//    private DcMotor leftBackMotor_1 = null;
//    private DcMotor rightBackMotor_2 = null;
//    private DcMotor rightFrontMotor_3 = null;
//
//
//    private double leftFrontPower;
//    private double rightFrontPower;
//    private double leftBackPower;
//    private double rightBackPower;
//
//
//    public void init(HardwareMap hwMap) {
//        leftFrontMotor_0 = hwMap.get(DcMotor.class, "leftFrontMotor_0");
//        leftBackMotor_1 = hwMap.get(DcMotor.class, "leftBackMotor_1");
//        rightBackMotor_2 = hwMap.get(DcMotor.class, "rightBackMotor_2");
//        rightFrontMotor_3 = hwMap.get(DcMotor.class, "rightFrontMotor_3");
//
//        leftFrontMotor_0.setDirection(DcMotor.Direction.FORWARD);
//        leftBackMotor_1.setDirection(DcMotor.Direction.FORWARD);
//        rightBackMotor_2.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontMotor_3.setDirection(DcMotor.Direction.FORWARD);
//    }

    public void controllerDrive(double axial, double lateral, double yaw){
        // Function takes in desired directions as parameters.

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;
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

        Board.setMotorPower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
        // Sends the desired inputs over to the Board class.

//        leftFrontMotor_0.setPower(leftFrontPower);
//        leftBackMotor_1.setPower(leftBackPower);
//        rightBackMotor_2.setPower(rightBackPower);
//        rightFrontMotor_3.setPower(rightFrontPower);





    }
}
