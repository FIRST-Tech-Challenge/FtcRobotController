///* Copyright (c) 2023 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package teamcode;
//
////import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
////import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
////import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
//import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
//import static java.lang.Math.abs;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external..Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//import java.util.concurrent.TimeUnit;
//
////@Disabled
//public class TeamAutoDrive extends Thread {
//    private HardwareMap hardwareMap;
//    private Telemetry telemetry;
//    private Gamepad gamepad1;
//    private TfodProcessor tfod;
//    private ElapsedTime runtime = new ElapsedTime();
//    public Robot robotInstance;// = new Robot(map, tel);
//
//    // Define class members
//    Servo armServo, trayServo; //clawServo;
//    Servo clawServo;
//   // CRServo clawServo, trayServo;
//
//    final int CAMERA_EXPOSURE = 15; // for bright day light set this to 3 and for low light like garage set it to 15
//    double tray_start_position = 0.4;
//    double tray_end_position = 0.7;
//    double arm_start_position = 0.0;
//    double claw_start_position = 1;
//
//    double arm_end_position = .65;
//    double claw_end_position = 0.5;
//    double claw_increment = .1;
//
//    static final double DRIVE_SPEED = 1;
//    static final double TURN_SPEED = 0.6;
//    float turn_distance = 24;
//    float team_object_distance = 25;
//    float forward_distance = (float)5.5;
//    float reverse_distance = (float)6.5;
//    float move_away_distance = 2;
//    double move_away_adjustment = 0;
//    float backdrop_cross_distance = (float)26.5;
//    float front_cross_distance = backdrop_cross_distance + 48;
//    final double DESIRED_DISTANCE = 5.5; //  this is how close the camera should get to the target (inches)
//    // Calculate the COUNTS_PER_INCH for your specific drive train.
//    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
//    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
//    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
//    // This is gearing DOWN for less speed and more torque.
//    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder 1440
//    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
//    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
//    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
//    final double WHEEL_WIDTH_INCHES = 3.3;
//    final double PARALLEL_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_WIDTH_INCHES * 3.1415);
//    private static final String TFOD_MODEL_ASSET = "TeamPropAbs0.tflite";//"MyModelStoredAsAsset.tflite";
//    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
//    // this is used when uploading models directly to the RC using the model upload interface.
//    private String TFOD_MODEL_FILE = "TeamPropAbs0.tflite";//"/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
//    // Define the labels recognized in the model for TFOD (must be in training order!)
//    private static final String[] LABELS = {
//            "Abs0",
//            "RedAbs0",
//    };
//    // Adjust these numbers to suit your robot.
//
//
//    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
//    //  applied to the drive motors to correct the error.
//    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
//    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double STRAFE_GAIN = 0.004;   // 0.015 Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    final double TURN_GAIN = 0.005;   // 0.01  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
//
//    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
//
//    private DcMotor leftFrontDrive = null;  //  Used to control the left front drive wheel
//    private DcMotor rightFrontDrive = null;  //  Used to control the right front drive wheel
//    private DcMotor leftBackDrive = null;  //  Used to control the left back drive wheel
//    private DcMotor rightBackDrive = null;  //  Used to control the right back drive wheel
//
//    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
//    private int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
//    private VisionPortal visionPortal;               // Used to manage the video source.
//    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
//    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
//
//
//    TeamAutoDrive(HardwareMap map, Telemetry tel, Gamepad pad, String tfod_model) {
//        gamepad1 = pad;
//        hardwareMap = map;
//        telemetry = tel;
//        robotInstance = new Robot(map, tel);
//        TFOD_MODEL_FILE = tfod_model;
//        initRobotSettings();
//    }
//
//    public void initRobotSettings() {
//
//        // initialize TFOD
//        initTfod();
//
//        // Initialize the Apriltag Detection process
//        initAprilTag();
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must match the names assigned during the robot configuration.
//        // step (using the FTC Robot Controller app on the phone).
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "motor_fl");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor_fr");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "motor_bl");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "motor_br");
//        armServo = hardwareMap.get(Servo.class, "armServo");
//        clawServo = hardwareMap.get(Servo.class, "clawServo");
//        trayServo = hardwareMap.get(Servo.class, "tray");
//
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftFrontDrive.setDirection(REVERSE);
//        leftBackDrive.setDirection(REVERSE);
//        rightFrontDrive.setDirection(FORWARD);
//        rightBackDrive.setDirection(FORWARD);
//
//
////        clawServo.setDirection(REVERSE);
////        clawServo.setPower(0.2);
////        try {
////            sleep(100);
////        } catch (InterruptedException e) {
////            throw new RuntimeException(e);
////        }
//        clawServo.setPosition(claw_start_position);
//        armServo.setPosition(arm_start_position);
//        trayServo.setPosition(tray_start_position);
//        //clawServo.setPower(0);
//    }
//
//    private boolean findAprilTag(int tag_id) {
//        try {
//            sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        boolean targetFound = false;    // Set to true when an AprilTag target is detected
//        // Step through the list of detected tags and look for a matching tag
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        for (AprilTagDetection detection : currentDetections) {
//            if ((detection.metadata != null) &&
//                    ((tag_id < 0) || (detection.id == tag_id))) {
//                targetFound = true;
//                desiredTag = detection;
//                telemetry.addData("Found Tag", "location %d", tag_id);
//                break;  // don't look any further.
//            } else {
//                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary, found %d\n", tag_id, detection.id);
//            }
//        }
//
//        // Tell the driver what we see, and what to do.
//        if (targetFound) {
//            telemetry.addData(">", "The distance to April tag is \n");
//            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
//            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
//            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
//            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
//            telemetry.addData("X", "%5.1f inches", desiredTag.ftcPose.x);
//            telemetry.addData("Y", "%3.0f degrees", desiredTag.ftcPose.y);
//            telemetry.addData("Z", "%3.0f degrees", desiredTag.ftcPose.z);
//        } else {
//            telemetry.addData(">", "Drive using joysticks to find valid target\n");
//        }
//        telemetry.update();
//        try {
//            sleep(500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        return targetFound;
//    }
//
//    public void driveToTeamAprilTag(int team_object_position) {
//
//        boolean targetFound = false;    // Set to true when an AprilTag target is detected
//        double desired_range = 0;
//        double wait_time = 0.0;
//        //set desired TAG ID depending on team object position
//        DESIRED_TAG_ID = team_object_position;
//        targetFound = findAprilTag(DESIRED_TAG_ID);
//
//        // drive using april tag code
//        // turn the robot if the april tag is at more than 0 degrees
//        if (targetFound && (Math.abs(desiredTag.ftcPose.yaw) > 0)) {
//            telemetry.addData("Fixing the turn", "correcting the turn %5.2f\n", desiredTag.ftcPose.yaw);
//            telemetry.update();
//            //turn robot by yaw degrees and make it straight
//            double temp_turn_distance = turn_distance * (desiredTag.ftcPose.yaw / 90);
//            driveRobot(TURN_SPEED, -temp_turn_distance, temp_turn_distance, 3.0);
//            wait_time = 1.5 * temp_turn_distance * 90;
//            // decide whether to move Right or Left based on yaw positive or negative
////            if (temp_turn_distance > 0) {
////                //moveParallelToRight((int) wait_time);
////                moveParallelToRightD(DRIVE_SPEED, desiredTag.ftcPose.x, 3.0);
////            } else {
////                // moveParallelToLeft((int) -wait_time);
////                moveParallelToLeftD(DRIVE_SPEED, desiredTag.ftcPose.x, 3.0);
////            }
//            targetFound = findAprilTag(DESIRED_TAG_ID);
//        }
//        // drive closer to april tag
////        if (targetFound) {
////            double x_distrance = desiredTag.ftcPose.x;
////            // if target found go closer and in-front of april tag
////            // use the encoder based drive function
////            desired_range = (desiredTag.ftcPose.y - 3 * DESIRED_DISTANCE);
////            driveRobot(DRIVE_SPEED, desired_range, desired_range, 3.0);
////            turnRobotOff();
////
////            if (x_distrance > 0) {
////                wait_time = x_distrance * 90;
////                //moveParallelToRight((int) wait_time);
////                moveParallelToRightD(DRIVE_SPEED, x_distrance, 3.0);
////            } else {
////                wait_time = -x_distrance * 90;
////                moveParallelToLeftD(DRIVE_SPEED, x_distrance, 3.0);
////                //moveParallelToLeft((int) wait_time);
////            }
////        }
//        try {
//            sleep(100);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        // Scan for April Tag again to be sure that you are closer
//        targetFound = findAprilTag(DESIRED_TAG_ID);
//        moveRobotUsingAprilTag(targetFound);
////        if (team_object_position == 3 || team_object_position == 6){
////            moveParallelToRightD(DRIVE_SPEED, 2, 2.0);
////        }
////        else if (team_object_position == 1 || team_object_position == 4){
////            moveParallelToLeftD(DRIVE_SPEED, 2, 2.0);
////        }
//    }
//
//    /**
//     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
//     */
//    public int teamObjectDetectionTfod() throws InterruptedException {
//        int location = -1; //  1 left and 2 - center and 3 - right
//        double angle = 0;
//        double x = -9999;
//        double y = -9999;
//        boolean found_pixel = false;
//        String object_label = "";
//        sleep(300);
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//        telemetry.update();
//        sleep(50);
//
//        // try again if we did not find object in first attempt
//        int repeat_count = 3;
//        if (currentRecognitions.size() == 0) {
//            while (repeat_count > 0) {
//                currentRecognitions = tfod.getRecognitions();
//                telemetry.addData("# Objects Detected",  "%d on next try %d", currentRecognitions.size(), repeat_count);
//                telemetry.update();
//                sleep(300);
//                if (currentRecognitions.size() > 0) {
//                    break;
//                }
//                repeat_count--;
//            }
//        }
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            x = (recognition.getLeft() + recognition.getRight()) / 2;
//            y = (recognition.getTop() + recognition.getBottom()) / 2;
//            angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
//            object_label = recognition.getLabel();
//            telemetry.addData("", " ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", object_label, recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Angle", ".0f", angle);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//            if (object_label.equals("Abs0") || object_label.equals("RedAbs0")) {
//                telemetry.addData("Found team object pixel, breaking", "%s", object_label);
//                found_pixel = true;
//                break;
//            } else {
//                telemetry.addData("Not Found white pixel, not breaking", "%s", object_label);
//            }
//
//        }   // end for() loop
//        if (found_pixel) {
//            telemetry.addData(">", "found pixel\n"); //enter 2 - Center, 1 left and 3 right
//            if (x > 0 && x < 150) {
//                // if x between 150 to 350 assume it to be left
//                location = 1;
//            } else if (x > 150 && x < 350) {
//                // if x between 150 to 350 assume it to be center
//                location = 2;
//            } else if (x > 400) {
//                // if x between 150 to 350 assume it to be right
//                location = 3;
//            }
//            telemetry.addData("Team element is at location", "%s", location);
//            telemetry.update();
//            sleep(50);
//        } // end of if
//        telemetry.update();
//        return location;
//    }   // end method telemetryTfod()
//
//    /**
//     * Move robot according to desired axes motions
//     * <p>
//     * Positive X is forward
//     * <p>
//     * Positive Y is strafe left
//     * <p>
//     * Positive Yaw is counter-clockwise
//     */
//    public void moveRobotUsingAprilTag(boolean targetFound) {
//        double drive = 0;        // Desired forward power/speed (-1 to +1)
//        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
//        double turn = 0;        // Desired turning power/speed (-1 to +1)
//        double range = 0;
//        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
//        //if (gamepad1.left_bumper && targetFound) {
//        if (targetFound) {
//            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//            range = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
//            double headingError = desiredTag.ftcPose.bearing;
//            double yawError = desiredTag.ftcPose.yaw;
//
//            // Use the speed and turn "gains" to calculate how we want the robot to move.
//            drive = Range.clip(range * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
//            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//        } else {
//
//            // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//            drive = -gamepad1.left_stick_y / 2.0;  // Reduce drive rate to 50%.
//            strafe = -gamepad1.left_stick_x / 2.0;  // Reduce strafe rate to 50%.
//            turn = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//            telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//        }
//
//
//        // Calculate wheel powers.
//        double leftFrontPower = drive - strafe - turn;
//        double rightFrontPower = drive + strafe + turn;
//        double leftBackPower = drive + strafe - turn;
//        double rightBackPower = drive - strafe + turn;
//        int sleep_time = 0;
//        sleep_time = (int) ((range * 25) / (1 - 2 * (MAX_AUTO_SPEED + 0.05 - (rightFrontPower + leftFrontPower) / 2)));
//
//        telemetry.addData("Auto", "Left Front %5.2f, Right Front %5.2f, Left Back %5.2f, , Right Back %5.2f ", leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
//        telemetry.addData("Sleep time", "time %5d and range %5.2f", sleep_time, range);
//        telemetry.update();
//        try {
//            sleep(50);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//
//        // Normalize wheel powers to be less than 1.0
//        double max = Math.max(abs(leftFrontPower), abs(rightFrontPower));
//        max = Math.max(max, abs(leftBackPower));
//        max = Math.max(max, abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        // Send powers to the wheels.
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//
//        try {
//            sleep(sleep_time);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        turnRobotOff();
//    }
//
//    public void turnRobotOff() {
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        rightBackDrive.setPower(0);
//        leftBackDrive.setPower(0);
//    }
//
//    public void moveParallelToLeftD(double speed, double distance, double timeoutS) {
//        //robotInstance.moveLeftToPosition(1.0, distance);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Starting at", "%7d :%7d",
//                leftFrontDrive.getCurrentPosition(),
//                rightFrontDrive.getCurrentPosition());
//        telemetry.update();
//        //sleep(250);
//        int newTarget;
//        //int newRightTarget;
//
//        // Determine new target position, and pass to motor controller
//
//        newTarget = (int) (distance * PARALLEL_COUNTS_PER_INCH); //COUNTS_PER_INCH);
//        telemetry.addData("Heading to", "%7d",
//                //newLeftTarget,
//                newTarget);
//        telemetry.update();
//        //newTarget = (int) (distance * COUNTS_PER_INCH);
//        //newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
//        telemetry.addData("Heading to", "%7d",
//                //newLeftTarget,
//                newTarget);
//        telemetry.update();
//        //sleep(1000);
//        leftFrontDrive.setTargetPosition((-1) * newTarget);
//        rightFrontDrive.setTargetPosition(newTarget);
//        leftBackDrive.setTargetPosition(newTarget);
//        rightBackDrive.setTargetPosition((-1) * newTarget);
//
//        // Turn On RUN_TO_POSITION
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // reset the timeout time and start motion.
//        runtime.reset();
//        rightFrontDrive.setPower(abs(speed));
//        leftFrontDrive.setPower( abs(speed));
//        leftBackDrive.setPower(abs(speed));
//        rightBackDrive.setPower( abs(speed));
//
//
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//        // its target position, the motion will stop.  This is "safer" in the event that the robot will
//        // always end the motion as soon as possible.
//        // However, if you require that BOTH motors have finished their moves before the robot continues
//        // onto the next step, use (isBusy() || isBusy()) in the loop test.
//        //opModeIsActive() &&
//        while (
//                (runtime.seconds() < timeoutS) &&
//                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {
//
//            // Display it for the driver.
//            telemetry.addData("Running to", " %7d", newTarget);
//            telemetry.addData("Currently at", " at %7d :%7d",
//                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
//            telemetry.update();
//            //sleep(500);   // optional pause after each move.
//        }
//
//        // Stop all motion;
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
////        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        try {
//            sleep(150);   // optional pause after each move.
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//    }
//
//    public void moveParallelToLeftTime(int sl_sec) {
//        int multiplication_factor = 3;
//        double power = .30 * multiplication_factor;
//        telemetry.addData("Driving", "to left for  %5d ", sl_sec / multiplication_factor);
//        telemetry.update();
//        rightFrontDrive.setPower((1) * power);
//        leftFrontDrive.setPower((-1) * power);
//        leftBackDrive.setPower((1) * power);
//        rightBackDrive.setPower((-1) * power);
//
//        try {
//            sleep(sl_sec/(multiplication_factor)); //+(long)0.5)
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        turnRobotOff();
//    }
//
//    public void moveParallelToRightD(double speed, double distance, double timeoutS) {
//        //robotInstance.moveLeftToPosition(1.0, distance);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Starting at", "%7d :%7d",
//                leftFrontDrive.getCurrentPosition(),
//                rightFrontDrive.getCurrentPosition());
//        telemetry.update();
//        //sleep(250);
//        int newTarget;
//        //int newRightTarget;
//
//        // Determine new target position, and pass to motor controller
//
//
//        newTarget = (int) (distance * PARALLEL_COUNTS_PER_INCH); //COUNTS_PER_INCH);
//        telemetry.addData("Heading to", "%7d",
//                //newLeftTarget,
//                newTarget);
//        telemetry.update();
//        //newTarget = (int) (distance * COUNTS_PER_INCH);
//        //newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
//        telemetry.addData("Heading to", "%7d",
//                //newLeftTarget,
//                newTarget);
//        telemetry.update();
//        //sleep(1000);
//        leftFrontDrive.setTargetPosition(newTarget);
//        rightFrontDrive.setTargetPosition((-1) * newTarget);
//        leftBackDrive.setTargetPosition((-1) * newTarget);
//        rightBackDrive.setTargetPosition(newTarget);
//
//        // Turn On RUN_TO_POSITION
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // reset the timeout time and start motion.
//        runtime.reset();
//        rightFrontDrive.setPower(abs(speed));
//        leftFrontDrive.setPower( abs(speed));
//        leftBackDrive.setPower(abs(speed));
//        rightBackDrive.setPower( abs(speed));
//
//
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//        // its target position, the motion will stop.  This is "safer" in the event that the robot will
//        // always end the motion as soon as possible.
//        // However, if you require that BOTH motors have finished their moves before the robot continues
//        // onto the next step, use (isBusy() || isBusy()) in the loop test.
//        //opModeIsActive() &&
//        while (
//                (runtime.seconds() < timeoutS) &&
//                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {
//
//            // Display it for the driver.
//            telemetry.addData("Running to", " %7d", newTarget);
//            telemetry.addData("Currently at", " at %7d :%7d",
//                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
//            telemetry.update();
//            //sleep(500);   // optional pause after each move.
//        }
//
//        // Stop all motion;
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
////        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        try {
//            sleep(150);   // optional pause after each move.
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//    }
//
//    public void moveParallelToRightTime(int sl_sec) {
//        int multiplication_factor = 3;
//        double power = .30 * multiplication_factor;
//        telemetry.addData("Driving", "to Right for  %5d ", sl_sec/multiplication_factor);
//        telemetry.update();
//        leftFrontDrive.setPower((1) * power);
//        rightFrontDrive.setPower((-1) * power);
//        rightBackDrive.setPower((1) * power);
//        leftBackDrive.setPower((-1) * power);
//        try {
//            sleep(sl_sec / (multiplication_factor)); //+(long)0.5
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        turnRobotOff();
//    }
//
//    /**
//     * Initialize the AprilTag processor.
//     */
//    private void initAprilTag() {
//        // Create the AprilTag processor by using a builder.
//        aprilTag = new AprilTagProcessor.Builder().build();
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        aprilTag.setDecimation(2);
//
//        // Create the vision portal by using a builder.
//        if (USE_WEBCAM) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessors(tfod, aprilTag)
//                    .build();
//            try {
//                setManualExposure(CAMERA_EXPOSURE, 250);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        } else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .addProcessors(aprilTag, tfod)
//                    .build();
//        }
//    }
//
//    /**
//     * Initialize the TensorFlow Object Detection processor.
//     */
//    private void initTfod() {
//
//        telemetry.addData("TFOD Model", "is %s", TFOD_MODEL_FILE);
//        telemetry.update();
//        // Create the TensorFlow processor by using a builder.
//        tfod = new TfodProcessor.Builder()
//
//                // Use setModelAssetName() if the TF Model is built in as an asset.
//                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                //.setModelAssetName(TFOD_MODEL_ASSET)
//
//                //set model file to trained model file
//                .setModelFileName(TFOD_MODEL_FILE)
//
//                .setModelLabels(LABELS)
//                //.setIsModelTensorFlow2(true)
//                //.setIsModelQuantized(true)
//                //.setModelInputSize(300)
//                //.setModelAspectRatio(1.0 / 1.0)
//                //.setCameraResolution(new Size(640, 480));
//
//                .build();
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        //builder.enableCameraMonitoring(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        // builder.addProcessor(tfod);
//
//        // Build the Vision Portal, using the above settings.
//        // visionPortal = builder.build();
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        //tfod.setMinResultConfidence(0.75f);
//
//        // Disable or re-enable the TFOD processor at any time.
//        //visionPortal.setProcessorEnabled(tfod, true);
//
//    }   // end method initTfod()
//
//    /*
//     Manually set the camera gain and exposure.
//     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
//    */
//    private void setManualExposure(int exposureMS, int gain) throws InterruptedException {
//        // Wait for the camera to be open, then use the controls
//
//        if (visionPortal == null) {
//            telemetry.addData("Vision Portal is null", "error");
//            telemetry.update();
//            sleep(200);
//            return;
//        }
//
//        // Make sure camera is streaming before we try to set the exposure controls
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addData("Camera", "Waiting");
//            telemetry.update();
//            //!isStopRequested() &&
//            while ((visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
//                sleep(20);
//            }
//            telemetry.addData("Camera", "Ready");
//            telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//            exposureControl.setMode(ExposureControl.Mode.Manual);
//            sleep(50);
//        }
//        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
//        sleep(20);
//        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
//        gainControl.setGain(gain);
//        sleep(20);
//        telemetry.addData("Set exposure to", "%7d :%7d",
//                exposureMS,
//                gain);
//        telemetry.update();
//        sleep(50);
//    }
//
//    /*
//     *  Method to perform a relative move, based on encoder counts.
//     *  Encoders are not reset as the move is based on the current position.
//     *  Move will stop if any of three conditions occur:
//     *  1) Move gets to the desired position
//     *  2) Move runs out of time
//     *  3) Driver stops the OpMode running.
//     */
//    public void driveRobot(double speed,
//                           double leftInches, double rightInches,
//                           double timeoutS) {
//
//
//        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Starting at", "%7d :%7d",
//                leftFrontDrive.getCurrentPosition(),
//                rightFrontDrive.getCurrentPosition());
//        telemetry.update();
//        //sleep(250);
//        int newLeftTarget;
//        int newRightTarget;
//
//        // Determine new target position, and pass to motor controller
//        newLeftTarget = (int) (leftInches * COUNTS_PER_INCH);
//        newRightTarget = (int) (rightInches * COUNTS_PER_INCH);
//        telemetry.addData("Heading to", "%7d :%7d",
//                newLeftTarget,
//                newRightTarget);
//        telemetry.update();
//
//        leftFrontDrive.setTargetPosition(newLeftTarget);
//        rightFrontDrive.setTargetPosition(newRightTarget);
//        leftBackDrive.setTargetPosition(newLeftTarget);
//        rightBackDrive.setTargetPosition(newRightTarget);
//
//        // Turn On RUN_TO_POSITION
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // reset the timeout time and start motion.
//        runtime.reset();
//        leftFrontDrive.setPower(abs(speed));
//        rightFrontDrive.setPower(abs(speed));
//        leftBackDrive.setPower(abs(speed));
//        rightBackDrive.setPower(abs(speed));
//
//        // keep looping while we are still active, and there is time left, and both motors are running.
//        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//        // its target position, the motion will stop.  This is "safer" in the event that the robot will
//        // always end the motion as soon as possible.
//        // However, if you require that BOTH motors have finished their moves before the robot continues
//        // onto the next step, use (isBusy() || isBusy()) in the loop test.
//        //opModeIsActive() &&
//        while (
//                (runtime.seconds() < timeoutS) &&
//                        (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {
//
//            // Display it for the driver.
//            telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
//            telemetry.addData("Currently at", " at %7d :%7d",
//                    leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
//            telemetry.update();
//            //sleep(500);   // optional pause after each move.
//        }
//
//        // Stop all motion;
//        leftFrontDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
////        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        try {
//            sleep(150);   // optional pause after each move.
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//    }
//
//    public void pushTrayPixel(long sleep_second, float fdistance, float rdistance) {
//            double currentTrayPos = tray_end_position;
//            trayServo.setPosition(tray_end_position);
//            driveRobot(DRIVE_SPEED, fdistance, fdistance, 2);
//            driveRobot(DRIVE_SPEED, -(rdistance), -(rdistance), 2);
//            trayServo.setPosition(tray_start_position);
//    }
//
//    public void dropPixel() {
//        try {
//            double currentArmPos = arm_start_position;
//            while (currentArmPos <= arm_end_position) {
//                armServo.setPosition(currentArmPos);
//                sleep(150);
//                telemetry.addData("Arm Position", "Current %5.2f End:%5.2f",
//                        currentArmPos,
//                        arm_end_position);
//                telemetry.update();
//                //sleep(1000);
//                currentArmPos = currentArmPos + 0.05;
//            }
//            armServo.setPosition(arm_end_position);
//            //sleep(1200);
//            //clawServo.setPosition(claw_start_position);
//            double currentClawPos = claw_start_position;
//            while (currentClawPos >= claw_end_position) {
//                clawServo.setPosition(currentClawPos);
//                sleep(150);
//                telemetry.addData("Claw Position", "Current %5.2f End:%5.2f",
//                        currentClawPos,
//                        claw_end_position);
//                telemetry.update();
//                //sleep(1000);
//                currentClawPos = currentClawPos - claw_increment;
//            }
//
////            clawServo.setPower(0);
////            sleep(100);
////            clawServo.setDirection(FORWARD);
////            sleep(300);
////            long drop_time = 1800;
////            long sleep_time = 100;
////
////            //sleep(drop_time);
////            while (drop_time > 0) {
////                clawServo.setPower(.2);
////                try {
////                    sleep(sleep_time);
////                } catch (InterruptedException e) {
////                    throw new RuntimeException(e);
////                }
////                drop_time = drop_time - sleep_time;
////                clawServo.setPower(0);
////                telemetry.addData("Claw Position", "Current %d End:%d",
////                        drop_time,
////                        drop_time);
////                telemetry.update();
////                sleep(1000);
////            }
////
////            clawServo.setDirection(REVERSE);
////            sleep(300);
////            while (drop_time > 0) {
////                clawServo.setPower(.2);
////                try {
////                    sleep(sleep_time);
////                } catch (InterruptedException e) {
////                    throw new RuntimeException(e);
////                }
////                drop_time = drop_time - sleep_time;
////                clawServo.setPower(0);
////            }
//
////            double currentClawPos = claw_start_position;
////            while (currentClawPos <= claw_end_position) {
////                clawServo.setPosition(currentClawPos);
////                sleep(150);
////                telemetry.addData("Claw Position", "Current %5.2f End:%5.2f",
////                        currentClawPos,
////                        claw_end_position);
////                telemetry.update();
////                //sleep(1000);
////                currentClawPos = currentClawPos + claw_increment;
////            }
//            //clawServo.setPosition(claw_end_position);
//            //sleep(1000);
//            // drive it back a little bit
//            //driveRobot(DRIVE_SPEED / 3, -5, -5, 1.0);
//
//            //sleep (500);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//    }
//
//    public void parkRobot(int april_tag_number) {
//
////        clawServo.setDirection(REVERSE);
////        clawServo.setPower(.2);
////        try {
////            sleep(200);
////        } catch (InterruptedException e) {
////            throw new RuntimeException(e);
////        }
////        clawServo.setPower(0);
//        clawServo.setPosition(claw_start_position);
//        armServo.setPosition(arm_start_position);
//        int move_time = 0;
//        int move_distance = 0;
//        if (april_tag_number == 1 || april_tag_number == 6){
//            move_time = 1900;
//            move_distance = 20;
//        } else if (april_tag_number == 4 || april_tag_number == 3){
//            move_time = 2700;
//            move_distance = 35;
//        } else {
//            move_time = 2400;
//            move_distance = 24;
//        }
//        if (april_tag_number <= 3) {
//            moveParallelToLeftD(DRIVE_SPEED/4, move_distance, 3.0);
//            //moveParallelToLeft(move_time);
//        } else {
//            //moveParallelToRight( move_time);
//            moveParallelToRightD(DRIVE_SPEED/4, move_distance, 3.0);
//        }
//        driveRobot(DRIVE_SPEED/4, 10, 10, 2.0);
//
//    }
//
//    public int tryAgainTeamObjectDetection() {
//        int new_object_position = -1;
//        int team_object_position = 2;
//        driveRobot(DRIVE_SPEED, team_object_distance / 2, team_object_distance / 2, 2.0);  // S1: Forward 24 Inches with 5 Sec timeout
//        try {
//            sleep(1000);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        // if object not found at start then try again
//        try {
//            new_object_position = teamObjectDetectionTfod();
//
//            if (new_object_position != -1) {
//                // object found at center place
//                team_object_position = 2;
//                driveRobot(DRIVE_SPEED, team_object_distance / 2, team_object_distance / 2, 2.0);
//                return team_object_position;
//            } else {
//                // turn left and see if you can detect it
//                driveRobot(TURN_SPEED, -turn_distance / 4, turn_distance / 4, 1.0);
//                new_object_position = teamObjectDetectionTfod();
//                if (new_object_position != -1) {
//                    // object found at left place
//                    team_object_position = 1;
//                    driveRobot(TURN_SPEED, turn_distance / 4, -turn_distance / 4, 1.0);
//                } else {
//                    // turn right and see if you can detect it
//                    driveRobot(TURN_SPEED, turn_distance / 2, -turn_distance / 2, 2.0);
//                    new_object_position = teamObjectDetectionTfod();
//                    if (new_object_position != -1) {
//                        // object found at left place
//                        team_object_position = 3;
//                    }
//                    driveRobot(TURN_SPEED, -turn_distance / 4, turn_distance / 4, 1.0);
//                }
//                driveRobot(DRIVE_SPEED, team_object_distance / 2, team_object_distance / 2, 2.0);
//                return team_object_position;
//            }
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//        //return 2;
//    }
//}
