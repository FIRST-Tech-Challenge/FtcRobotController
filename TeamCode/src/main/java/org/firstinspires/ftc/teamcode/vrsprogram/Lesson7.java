package org.firstinspires.ftc.teamcode.vrsprogram;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Lesson7", group = "Tutorial")
public class Lesson7 extends LinearOpMode {
    // create DcMotor objects
    private DcMotor[] wheels = new DcMotor[4];
    private String[] wheelNames = {"frontRight", "frontLeft", "backRight", "backLeft"};
    private IMU imu ;

    @Override
    public void runOpMode() {
        // initialize hardware variables
        for (int i = 0; i < wheels.length; i++) {
            wheels[i] = hardwareMap.get(DcMotor.class, wheelNames[i]);
        }
        // does not work in VRS yet
//        imu = hardwareMap.get(IMU.class, "imu");

        // Set the right motors in reverse and left in forward direction
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setDirection((i & 1) == 0 ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        }

        waitForStart();

        // when started
        if (opModeIsActive()) {
            moveToPosition(10000, 0.9);

            // turn left
            turn(3000);

            // set target position
            moveToPosition(8500, 0.9);
        }
    }

    private void turn(int degree) {
        for (int i = 0; i < wheels.length; i++) {
            wheels[i].setPower((i & 1) == 0 ? 1 : -1);
        }
        sleep(degree);
    }

    /**
     * @param position set target position
     * @param on       define variable for easier programming
     */
    private void moveToPosition(int position, double on) {
        wheels[1].setTargetPosition(position);

        // set mode to STOP_AND_RESET_ENCODER to set position to zero
        for (DcMotor motor : wheels) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // set mode to RUN_TO_POSITION
        for (DcMotor motor : wheels) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // loop until reach target position
        while (wheels[1].getCurrentPosition() < wheels[1].getTargetPosition()) {
            for (DcMotor motor : wheels) {
                motor.setPower(on);
            }
            // displays encoder position
            telemetry.addData("Position", wheels[1].getCurrentPosition());
            telemetry.update();
            sleep(50);
        }

        for (DcMotor motor : wheels) {
            motor.setPower(0);
        }
        sleep(100);
    }
}
