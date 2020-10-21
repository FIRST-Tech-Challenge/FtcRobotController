package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "EncoderAuto", group = "DreamMachines")
public class AutoWithEncoders extends LinearOpMode {

    public DMHardware robot = new DMHardware(true);

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 40;
    static final double WHEEL_DIAMETER_CM = 8.89;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.3;

    @Override
    public void runOpMode() {
        robot.initTeleOpIMU(hardwareMap);
        robot.timer.reset();

        waitForStart();

        encoderDrive(0.5, 30, 30, 30);
        encoderDrive(-0.5, 30, 30, 30);
        encoderDrive(0.5,90, 90, 30);
        encoderDrive(-0.5, 90, 90, 30);
    }


    // Method to run something with Encoders
    public void encoderDrive(double speed,double rightCM, double leftCM, double timeout) {

        int newLeftTarget;
        int newRightTarget;

        if(opModeIsActive()) {
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int)(-1 * (leftCM * COUNTS_PER_CM));
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int)(-1 * (rightCM * COUNTS_PER_CM));

            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.timer.reset();

            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);



            while (opModeIsActive() && robot.timer.seconds() < timeout && robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {
                telemetry.addData("Path", "Going to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Currently at %7d :&7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());

                telemetry.update();
            }

            robot.setPowerOfAllMotorsTo(0);

            // Reset all motors
            robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
}