package org.firstinspires.ftc.teamcode.mechanism;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ProgrammingBoard {
    private static DcMotor leftFrontMotor_0 = null;
    private static DcMotor leftBackMotor_1 = null;
    private static DcMotor rightBackMotor_2 = null;
    private static DcMotor rightFrontMotor_3 = null;

    // Defines the motors.


    public void init(HardwareMap hwMap) {
        // Function runs during init phase of robot.

        leftFrontMotor_0 = hwMap.get(DcMotor.class, "leftFrontMotor_0");
        leftBackMotor_1 = hwMap.get(DcMotor.class, "leftBackMotor_1");
        rightBackMotor_2 = hwMap.get(DcMotor.class, "rightBackMotor_2");
        rightFrontMotor_3 = hwMap.get(DcMotor.class, "rightFrontMotor_3");
        // Maps motors.

        leftFrontMotor_0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Sets the mode. *See official FTC guide for more options.

        leftFrontMotor_0.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor_1.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor_2.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor_3.setDirection(DcMotor.Direction.FORWARD);
        // Sets the direction.
    }

    public void setMotorPower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower){
        leftFrontMotor_0.setPower(leftFrontPower);
        leftBackMotor_1.setPower(leftBackPower);
        rightBackMotor_2.setPower(rightBackPower);
        rightFrontMotor_3.setPower(rightFrontPower);
        // Method allows for other classes to set the speed of each motor.
    }





}
