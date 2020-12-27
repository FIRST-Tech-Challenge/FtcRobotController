package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="fullFieldAutoUlt2020", group="Pushbot")
public class autoFullField extends LinearOpMode{

    //OpMode members
autoDeclarations robot = new autoDeclarations();
    //Timer
    private ElapsedTime  runtime = new ElapsedTime();

    //Constants
    static final double COUNTS_PER_MOTOR_REV = 1440; //Counts to rotations, testing later
    static final double DRIVE_GEAR_REDUCTION = 1.0; //If gears are added
    static final double WHEEL_DIAMETER_INCHES = 4.0; //Wheel size
    static final double CIRCUMFERENCE = Math.PI*WHEEL_DIAMETER_INCHES; //Circumference of wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION)/ CIRCUMFERENCE; //Converting counts to inches

    public void runOpMode() {
        //Initialize variables
        robot.init(hardwareMap);
        //telemetry message
        telemetry.addData("Status", "Resetting Encoders");
        robot.leftFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.leftFoward.getCurrentPosition(),
                robot.rightReverse.getCurrentPosition(),
                robot.leftReverse.getCurrentPosition(),
                robot.rightFoward.getCurrentPosition());
                telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(80, 80, 1.0);  //  Forward 80 Inches with 1 Sec timeout
        robot.intake.setPower(1); //Shoot rings
    }
        //encoderDrive variable
    public void encoderDrive(double leftInches, double rightInches, double timeoutS) {
int newLeftTarget;
int newRightTarget;


//Checking Opmode
if(opModeIsActive()) {
    //Find the new position
    newLeftTarget = robot.leftFoward.getCurrentPosition() + (int)(leftInches*COUNTS_PER_INCH);
            newRightTarget = robot.rightReverse.getCurrentPosition() + (int)(rightInches*COUNTS_PER_INCH);
            newLeftTarget = robot.leftReverse.getCurrentPosition() + (int)(leftInches*COUNTS_PER_INCH);
            newRightTarget = robot.rightFoward.getCurrentPosition() + (int)(rightInches*COUNTS_PER_INCH);

            //Turn on RUN_TO_POSITION
            robot.leftFoward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightReverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftReverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightFoward.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    runtime.reset();
    robot.leftFoward.setPower(Math.abs(speed));
    robot.rightReverse.setPower(Math.abs(speed));
    robot.leftReverse.setPower(Math.abs(speed));
    robot.rightFoward.setPower(Math.abs(speed));

    while(opModeIsActive() &&
            (runtime.seconds() < timeoutS) &&
            (robot.leftFoward.isBusy() || robot.rightReverse.isBusy() ||
             robot.leftReverse.isBusy() || robot.rightFoward.isBusy())) {
        telemetry.addData("Path1", "Running to %7d: %7d", newLeftTarget, newRightTarget);
        telemetry.addData("Path2", "Running at %7d: %7d",
                robot.leftFoward.getCurrentPosition(),
                robot.rightReverse.getCurrentPosition(),
                robot.leftReverse.getCurrentPosition(),
                robot.rightFoward.getCurrentPosition());
        telemetry.update();
    }
}



        robot.leftFoward.setPower(0);
        robot.rightReverse.setPower(0);
        robot.leftReverse.setPower(0);
        robot.rightFoward.setPower(0);
        robot.intake.setPower(0);

        robot.leftFoward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightReverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftReverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFoward.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}