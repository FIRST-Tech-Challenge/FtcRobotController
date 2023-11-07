package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
public class Manual  extends LinearOpMode{

    private RobotHardware robot = new RobotHardware(this);
    private double elbowPosition= 0;
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
            manageLauncher();
            manageArm();
            //manageGrabber();

            //temp code
            if (robot.getDistanceFromObject() < 5){
                //robot.driveRobot(0,1);
            }
            telemetry.addData("Distance Sensor",  robot.getDistanceFromObject());
            telemetry.addData("Right Color Sensor (blue)", robot.getRightColorSensorData().blue);
            telemetry.addData("Right Color Sensor (red)", robot.getRightColorSensorData().red);
            telemetry.addData("Left Color Sensor (blue)", robot.getLeftColorSensorData().blue);
            telemetry.addData("Left Color Sensor (red)", robot.getLeftColorSensorData().red);

            // April Tag detection call
            //telemetryAprilTag();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }

    private void manageDriveMotors(){
        double drive        = 0;
        double turn         = 0;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
        // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
        // This way it's also easy to just drive straight, or just turn.
        drive = gamepad2.right_stick_y;
        turn  = gamepad2.right_stick_x;

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

        telemetry.addData("Drive", "Left Stick");
        telemetry.addData("Turn", "Right Stick");
        telemetry.addData("-", "-------");

        telemetry.addData("Drive Power", "%.2f", drive);
        telemetry.addData("Turn Power",  "%.2f", turn);
        telemetry.update();
    }

    private void manageLauncher(){
        if (gamepad1.a){
            robot.resetLaunch();
        }

        if (gamepad1.b || gamepad1.right_bumper){
            robot.setLauncher(RobotHardware.LAUNCHER_MAX);
        }
    }
    private void manageArm(){
        double armDrive = gamepad1.right_stick_y;
        //double elbowDrive = gamepad1.left_stick_y;
        double grabberDrive =  gamepad1.left_stick_x;

        robot.moveArm(armDrive);
        /*
        if (elbowDrive < 0){
            robot.moveElbow(true);
        }
        else if (elbowDrive > 0) {
            robot.moveElbow(false);
        }
        */
        if (grabberDrive < 0) {
            robot.moveGrabber(true);
        }
        else if (grabberDrive > 0) {
            robot.moveGrabber(false);
        }

        if (gamepad1.dpad_up) {
            robot.moveElbow(true);
        }
        else if (gamepad1.dpad_down) {
            robot.moveElbow(false);
        }
        else{
            //robot.stopElbow();
        }


        /*
        if (gamepad1.dpad_up) {
            robot.moveArm(true);
        }
        else if (gamepad1.dpad_down) {
            robot.moveArm(false);
        }
        else{
            robot.stopArm();
        }

        //temp
        robot.setElbow(elbowPosition);
        elbowPosition += .1;
        */
    }

    private void manageGrabber(){
        /*
        if (gamepad1.dpad_left) {
            robot.moveGrabber(true);
        }
        else if (gamepad1.dpad_right) {
            robot.moveGrabber(false);
        }
        else{
            //robot.stopElbow();
        }
        */
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
