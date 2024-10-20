package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
public class Manual  extends LinearOpMode{
    private RobotHardware robot = new RobotHardware(this);
    private boolean isSlow = false;

    @Override
    public void runOpMode() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();
        robot.initCamera();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Status", "Ready to run...");
            telemetry.update();

            manageDriveMotors();
            manageGrabber();
            manageArm();


            telemetryAprilTag();

            robot.setViperSlideMotorTargetPosition();
            robot.setViperSlideMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(gamepad1.left_stick_y > 0){

                robot.setViperSlideDirectionForward();
                telemetry.addData("Status", "Left Joystick Moved up");
                robot.setViperSlideMotorPower(0.5);
            }
            if(gamepad1.left_stick_y < 0){
                robot.setViperSlideDirectionReverse();
                telemetry.addData("Status", "Left Joystick Moved down");
                robot.setViperSlideMotorPower(0.5);
            }



            if(gamepad1.left_stick_y==0){
                robot.setViperSlideMotorPower(0);
            }
            //below is the arm
           /* if(gamepad1.right_stick_y > 0){
                robot.setArmDirectionForward();
                telemetry.addData("Status", "Right Joystick Moved up");
                robot.setArmPower();
            }
            if(gamepad1.right_stick_y < 0){
                robot.setArmDirectionReverse();
                telemetry.addData("Status", "Right Joystick Moved down");
                robot.setArmPower();
            }
            if(gamepad1.right_stick_y==0){
                robot.setArmMotorPowerZero();
            }
*/

            if(gamepad2.right_bumper){
                //robot.goDiagonal(1);
                robot.goStrafe(-1);
            }
            else if (gamepad2.left_bumper){
                robot.goStrafe(1);

            }

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
            telemetry.update();
        }
    }

    private void manageDriveMotors(){
        double drive        = 0;
        double turn         = 0;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = gamepad2.right_stick_x;
        turn  = gamepad2.right_stick_y;

        //if button on gamepad2 is pressed
        //then drive and turn should be half
        if (gamepad2.a){
            isSlow = true;
        }
        if (gamepad2.b){
            isSlow = false;
        }

        if (isSlow){
            drive = drive / 2;
            turn = turn / 2;
        }


        // Combine drive and turn for blended motion. Use RobotHardware class
        robot.driveRobot(drive, turn);

        telemetry.addData("Drivexxxxx", "Left Stick");
        telemetry.addData("Turn", "Right Stick");
        telemetry.addData("-", "-------");

        telemetry.addData("Drive Power", "%.2f", drive);
        telemetry.addData("Turn Power",  "%.2f", turn);
        telemetry.update();
    }

    private void manageGrabber(){

        telemetry.addData("Gabber Key", gamepad1.right_stick_x);
        if (gamepad1.right_stick_x > 0) {
            robot.moveGrabber(true);
        }
        else if (gamepad1.right_stick_x < 0) {
                robot.moveGrabber(false);
        }
    }

    private void manageArm(){

        telemetry.addData("Arm Key", gamepad2.left_stick_x);
        telemetry.addData("Arm Position", robot.getArmServoPosition());
        if (gamepad2.left_stick_x > 0) {

            robot.moveArm(true);
            telemetry.addData("true", gamepad2.left_stick_x);
        }
        else if (gamepad2.left_stick_x < 0) {
            robot.moveArm(false);

            telemetry.addData("false", gamepad2.left_stick_x);
        }
    }



    private void telemetryAprilTag(){
        List<AprilTagDetection> currentDetections = robot.getAprilTag().getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

            if (detection.ftcPose.y <= 5){
                telemetry.addData("Status", "Running away...");
                robot.driveRobot(-0.5, 0);
            }
        }   // end for() loop
    }
}
