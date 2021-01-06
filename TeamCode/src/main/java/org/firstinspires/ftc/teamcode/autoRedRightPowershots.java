package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name = "RedRightPowershotsUlt2020", group = "Pushbot")
public class autoRedRightPowershots extends LinearOpMode {

    //OpMode members
    autoDeclarations robot = new autoDeclarations();
    //Timer
    private ElapsedTime runtime = new ElapsedTime();


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
        robot.encoderDrive(1.0, 70.75, 70.75, 4);  //Forward 22.75 Inches with 1 Sec timeout
        robot.turnDrive(1.0, -3.63, 7.0); //Turn counter clockwise 90 degrees
        robot.intake.setPower(1.0);//Shoot ring
        robot.turnDrive(1.0, -5.98, 7.0); //Turn counter clockwise 90 degrees
        robot.intake.setPower(1.0);//Shoot ring
        robot.turnDrive(1.0, -5.98, 7.0); //Turn counter clockwise 90 degrees
        robot.intake.setPower(1.0);//Shoot ring



    }
}