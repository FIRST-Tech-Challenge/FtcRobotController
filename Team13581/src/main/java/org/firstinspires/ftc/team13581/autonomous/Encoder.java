package org.firstinspires.ftc.team13581.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.team13581.RobotHardware;

@Autonomous(name="Encoder", group="Robot")
public class Encoder extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);


    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        waitForStart();
        // Step  through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        robot.encoderDrive(robot.DRIVE_SPEED, -18, -18, -18, -18, 5);
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 2) {
            robot.vSlidePosition = robot.VSLIDE_SCORE_SAMPLE_HIGH;
            robot.setvSlideDrivePosition();
        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 2) {
            robot.intake2.setPosition(1.0 / 10);
        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1) {
            robot.intake2.setPosition(0);
        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 2) {
            robot.vSlidePosition = robot.VSLIDE_START_POSITION;
            robot.setvSlideDrivePosition();
        }
        resetRuntime();

        robot.encoderDrive(robot.DRIVE_SPEED, 11, 11, 11, 11, 11);

        robot.encoderDrive(robot.DRIVE_SPEED, -41.25, 41.25, -41.25, 41.25, 5);

        robot.encoderDrive(robot.DRIVE_SPEED, -80, -40, 80, 40, 5);

        robot.encoderDrive(robot.DRIVE_SPEED, -2, -2, -2, -2, 5);
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1) {
            robot.intake2.setPosition(1.0 / 10);
        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1) {

            robot.leftWrist.setPosition(0.55);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 2) {

            robot.intake.setPower(robot.INTAKE_COLLECT);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1) {

            robot.leftWrist.setPosition(-0.55);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 0.25) {

            //robot.leftSlide.setPower(robot.RIGHT_SLIDE_EXTEND);
            //robot.rightSlide.setPower(robot.LEFT_SLIDE_EXTEND);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 0.1) {

            //robot.leftSlide.setPower(0.0);
            //robot.rightSlide.setPower(0.0);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1) {

            robot.intake2.setPosition(0);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 2) {

            robot.intake.setPower(1.0);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1){

            robot.intake.setPower(0.0);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1) {

            robot.encoderDrive(robot.DRIVE_SPEED, 24, 12, -24, -12, 5);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1){

            robot.encoderDrive(robot.DRIVE_SPEED, -32, -32, -32, -32, 5);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1) {

            robot.encoderDrive(robot.DRIVE_SPEED, -8, 8, -8, 8, 8);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 2) {
            robot.vSlidePosition = robot.VSLIDE_SCORE_SAMPLE_HIGH;
            robot.setvSlideDrivePosition();
        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 2) {
            robot.intake2.setPosition(1.2 / 10);
        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 1) {

            robot.intake2.setPosition(0);

        }
        resetRuntime();

        while (opModeIsActive()&&getRuntime() < 2) {
            robot.vSlidePosition = robot.VSLIDE_START_POSITION;
            robot.setvSlideDrivePosition();
        }
        resetRuntime();

    }

    }





/*robot.intake2.setPosition(1.0/9);
        robot.encoderDrive(0.55,15,15,15,15,5);
        robot.encoderDrive(0.55,-6,6,6,-6,5);
        robot.encoderDrive(0.55,75,37.5,-75,-37.5,5);
        robot.encoderDrive(0.55,36,-36,-36,36,5);
        robot.encoderDrive(0.55,-4,-4,-4,-4,5);
        robot.leftWrist.setPosition(0.572);
        robot.intake.setPower(robot.INTAKE_COLLECT);
sleep(500);
        robot.encoderDrive(0.15,6.5,6.5,6.5,6.5,5);
sleep(500);
        robot.leftWrist.setPosition(0.004);
        robot.intake.setPower(0);
        robot.encoderDrive(0.55,53,26.5,-53,-26.5,5);
        robot.intake2.setPosition(0);
sleep(250);
        robot.intake.setPower(robot.INTAKE_DEPOSIT);
sleep(1500);
robot.vSlidePosition = robot.VSLIDE_SCORE_SAMPLE_HIGH;
        robot.setvSlideDrivePosition();
        robot.intake.setPower(0);
        robot.encoderDrive(0.7,-32,-32,-32,-32,5);
        robot.intake2.setPosition(1.0 / 9);
sleep(1275);
        robot.intake2.setPosition(0);
sleep(100);
        robot.setvSlideDrivePosition();
robot.vSlidePosition = robot.VSLIDE_START_POSITION;
        robot.setvSlideDrivePosition();


//2nd sample is scored, 3rd sample is now in the process of being scored

        robot.encoderDrive(0.7,26,26,26,26,5);
        robot.intake2.setPosition(0);
sleep(50000);
//robot.encoderDrive(0.55,-50.5,-25.25,50.5,25.25,5);
//robot.encoderDrive(0.55,6.25,-6.25,-6.25,6.25,5);
// sleep(250);
// robot.leftWrist.setPosition(0.575);
//robot.encoderDrive(0.55,-6,-6,-6,-6,5);
//robot.intake.setPower(robot.INTAKE_COLLECT);
//robot.encoderDrive(0.25,8.5,8.5,8.5,8.5,5);
//sleep(500);
//robot.intake.setPower(0);
//robot.leftWrist.setPosition(0.004);
///sleep(250);
//robot.intake2.setPosition(0);
//sleep(750);
//robot.intake.setPower(robot.INTAKE_DEPOSIT);
//robot.encoderDrive(0.7,49,24.5,-49,-24.5,5);
//sleep(500);
//robot.intake.setPower(0);
//robot.vSlidePosition = robot.VSLIDE_SCORE_SAMPLE_HIGH;
//robot.setvSlideDrivePosition();
//robot.encoderDrive(0.7,-27.5,-27.5,-27.5,-27.5,5);
//robot.intake2.setPosition( 1.0/9);
//sleep(1275);
//robot.intake2.setPosition(0);
//sleep(100);
//robot.setvSlideDrivePosition();
//robot.vSlidePosition = robot.VSLIDE_START_POSITION;
//robot.setvSlideDrivePosition();
sleep(50000);*/



