package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Rams_Test_Auto_File", group="Pushbot")
//@Disabled
public class Rams_Test_Auto_File extends LinearOpMode {
    /* Declare OpMode members. */
    Team8648HardwarePushbot robot   = new Team8648HardwarePushbot();   // Use a Pushbot's hardware

    // Time counters and max time allowed
    private static final ElapsedTime runtime  = new ElapsedTime(); // a "stop watch" for encoderDrive so if robot does get to its tagret it stops after the "timeout" seconds
    public ElapsedTime tfTime   = new ElapsedTime(); // "stop" watch to give Tensor flow a couple seconds to compute after you hit RUN
    public static double tfSenseTime          = 5; // needs a couple seconds to process the image and ID the target. Shorten by one second at a time
    // Converts human terms of "inch" into robot language of "ticks" or "counts"
    static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // Varibles used for the encoderDrive default speed.
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;


    // Vuforia Stuff
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public String StackSize = "None";

    public enum WobbleTargetZone {
        BLUE_A,
        BLUE_B,
        BLUE_C

    }

    WobbleTargetZone Square = WobbleTargetZone.BLUE_A; // Default target zone


    private static final String VUFORIA_KEY =
            "AdzQ2ID/////AAABmThO+al+t0bqpiRTJ7x7MCQa37ZKlqlToYy/JxYbErfT1+jNdP8BvYT/juE2rfYLabYOaNlzqQ7UlLE547Rv+5aUeLWwuEoLOpa+7XradL3bwHmEmysPH7hD8jYnuZqFdVKvw/IuRkfQ664KpZPwLE3coupFkk3O0JANWUpeIBK4zssHrxDDhxJTpE3Fz1rTjxIWRO26tjTuYhHXN6affzAakoe6ZxhhfqrUFFJLYIUFWVQE6ABb2OCJ1UNb6txTXU15v2sjh936RZQDlqMce8rMUpLOFOjQt6K0nvYbHmY/u8yWSqoYdFNXJ5s2bkZvxXEJdGS6cf8JOHV/k+XrHGc7CVpXeCPQsDb8+C02U7wK";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;




    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            //tfod.setZoom(2.5, 1.78); // you will proably have to uncomment this becase the camera is so far from the stack that you have to zoom in.
        }

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();



        /*flMotor  = hwMap.get(DcMotor.class, "FLmotor");
        rlMotor = hwMap.get(DcMotor.class, "RLmotor");
        frMotor  = hwMap.get(DcMotor.class, "FRmotor");
        rrMotor = hwMap.get(DcMotor.class, "RRmotor");*/


        //robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rrMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        //robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.flMotor.getCurrentPosition(),
                robot.rlMotor.getCurrentPosition(),
                robot.frMotor.getCurrentPosition(),
                robot.rrMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (tfTime.time() < tfSenseTime && opModeIsActive()) { // need to let TF find the target so timer runs to let it do this
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        ///
                        StackSize = recognition.getLabel();
                        //telemetry.addData("Target", Target);
                        if (StackSize == "Quad") {
                            Square = WobbleTargetZone.BLUE_C;
                            telemetry.addData("Square", Square);
                        } else if (StackSize == "Single") {
                            Square = WobbleTargetZone.BLUE_B;
                            telemetry.addData("Square", Square);

                        }

                    }
                    telemetry.update();
                }
            }
            if (tfod != null) {
                tfod.shutdown(); // kills TF if you stop the opmode. Prevents crashing
            }
        }


        runtime.reset(); // need to reset the "drive" stopwatch so the encodeDrive timer is at zero when we start moving.
        // Switch case to handle each of the 3 options that the TF identifies
        switch(Square) {
            case BLUE_A: // no rings. 3 tiles (24 inches per tile) forward and one tile to the left from start
                telemetry.addData("Going to RED A", "Target Zone"); // change the message of you are going to BLUE to avoid confusion
                encoderDrive(DRIVE_SPEED, 73, 73, 2);
                encoderDrive(TURN_SPEED, 5, -5, 2);
                sleep(7000);
                robot.wc.setPosition(1);
                sleep(3000);
                telemetry.addData("Claw open", "Complete");
                sleep(500);
                break;
            case BLUE_B: // one ring  4 tiles straight ahead
                telemetry.addData("Going to RED B", "Target Zone");
                runtime.reset();
                encoderDrive(DRIVE_SPEED, 90,90,2);
                encoderDrive(TURN_SPEED, -5, 5, 2);
                sleep(1000);
                robot.wc.setPosition(1);
                sleep(1000);
                runtime.reset();
                break;
            case BLUE_C: // four rings. 5 tiles forward and one tile to the left.
                telemetry.addData("Going to RED C", "Target Zone");
                runtime.reset();
                encoderDrive(DRIVE_SPEED, 123, 123, 2);
                encoderDrive(TURN_SPEED,5,-5, 2);
                sleep(7000);
                robot.wc.setPosition(1);
                runtime.reset();
                //wobble.GripperOpen();
                //wobble.ArmExtend();
                //sleep(1000);
                //drivetime.reset();
                //gyroDrive(DRIVE_SPEED, -48.0, 0, 5);
                break;
        }




        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *
     *
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTargetRear;
        int newRightTargetRear;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            //newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            newLeftTarget = robot.flMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftTargetRear = robot.rlMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.frMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightTargetRear = robot.rrMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);





            robot.flMotor.setTargetPosition(newLeftTarget);
            robot.rlMotor.setTargetPosition(newLeftTargetRear);
            robot.frMotor.setTargetPosition(newRightTarget);
            robot.rrMotor.setTargetPosition(newRightTargetRear);

            // Turn On RUN_TO_POSITION
            robot.flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rrMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.flMotor.setPower(Math.abs(speed));
            robot.rlMotor.setPower(Math.abs(speed));
            robot.frMotor.setPower(Math.abs(speed));
            robot.rrMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.flMotor.isBusy() && robot.frMotor.isBusy() && robot.rlMotor.isBusy() && robot.rrMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.flMotor.getCurrentPosition(),
                        robot.rlMotor.getCurrentPosition(),
                        robot.frMotor.getCurrentPosition(),
                        robot.rrMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.flMotor.setPower(0);
            robot.rlMotor.setPower(0);
            robot.frMotor.setPower(0);
            robot.rrMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rrMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}