package org.firstinspires.ftc.teamcode.robots.catbot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.pipeline.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;

@Autonomous(name="Iron Giant OpMode", group="Challenge")
public class TestOpMode extends OpMode {
    //autonomous variables

    private boolean auton = false;
    private boolean goto1 = false;
    private boolean needsReset = true;
    private boolean turning = false;
    private boolean turningDone = false;
    private boolean turningDone1 = false;
    private boolean moving = false;
    private boolean movingDone = false;
    private boolean movingDone1 = false;
    private boolean movingDone2 = false;
    private boolean swivel = false;
    private boolean coneDown = false;
    public boolean shouldCone = true;
    private boolean forwardDone = false;
    private boolean strafeDone = false;
    private boolean shouldStrafe = true;
    private boolean red = true;
    private boolean swivelDone = false;
    private boolean shouldSwivel = true;
    private boolean motorsReset = false;
    private boolean shouldTurn = true;
    private boolean turnFinished = false;
    int tagDetected = 0;
    //vision variables
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    // UNITS ARE PIXELS
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.045; //tag size on iron reign signal sleeve
    int ID_TAG_OF_INTEREST = 1; // Tag ID 1 from the 36h11 family
    int tagCount = 0;
    boolean tagFound = false;
    AprilTagDetection tagOfInterest = null;
    //motors
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackRight = null;
    private DcMotorEx elevator = null;
    private Servo claw = null;
    // regular drive
    private double powerLeft = 0;
    private double powerRight = 0;
    // mecanum types
    private double powerFrontLeft = 0;
    private double powerFrontRight = 0;
    private double powerBackLeft = 0;
    private double powerBackRight = 0;
    //number variables
    private static final float DEADZONE = .1f;
    private static final int MAXELEVTICS = 4320;
    private static final int MINELEVTICS = 0;
    private int currElevTics = 0;
    private final double MOTORSTALLVALUE = .7;
    //boolean variables
    private boolean calibrate = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing " + this.getClass() + "...");
//        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();
        motorInit();
        visionInit();
    }

    @Override
    public void init_loop() {
//        if (!calibrate)
//            calib();
//        else
            aprilTagInitLoop();
            if(tagDetected == 2 || tagDetected == 3)
                turning = true;
        telemetry.update();

    }

    @Override
    public void loop() {
        telemetryOutput();
        tankDrive();
        if (auton) {
            autonDrive();
        } else {
//            mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//           if(calibrate)
                elevatorMove();
//            else
//                calib();
            clawMove();
        }
    }

    public void telemetryOutput() {
        telemetry.addData("is auton", auton);
        telemetry.addData("Elevator Position \t", elevator.getCurrentPosition());
        telemetry.addData("Claw Position \t", claw.getPosition());
        telemetry.addData("Back Right Position \t", motorBackRight.getCurrentPosition());
        telemetry.addData("Back Left Position \t", motorBackLeft.getCurrentPosition());
        telemetry.addData("Front Right Position \t", motorFrontRight.getCurrentPosition());
        telemetry.addData("Front Left Position \t", motorFrontLeft.getCurrentPosition());

    }

    public void strafe() {
        if (red) {
            if (motorFrontLeft.getCurrentPosition() < 2600)
                mechanumDrive(0, 1, 0);
            else
                strafeDone = true;
        }
        if (!red) {
            if (motorFrontLeft.getCurrentPosition() > -3000)
                mechanumDrive(0, -1, 0);
            else
                strafeDone = true;
        }
    }

    public void swivel() {
        if (red) {
            swivelDone = true;
            shouldSwivel = false;
        }
        if (!swivelDone) {
            if (motorFrontLeft.getCurrentPosition() < 1000)
                mechanumDrive(0, 0, 0.5);
            else {
                swivelDone = true;
            }
        }
    }
    public void turnNoStrafe()
        {
            if(!goto1){
            if (red) {
                if (motorFrontLeft.getCurrentPosition() < 1000 && shouldTurn)
                    mechanumDrive(0, 0, 0.5);
                else {
                    if (motorFrontLeft.getCurrentPosition() < 3500)
                        mechanumDrive(1, 0, 0);
                    else
                        mechanumDrive(0, 0, 0);
                }
            } else {
                if (motorFrontLeft.getCurrentPosition() < -1000 && shouldTurn)
                    mechanumDrive(0, 0, -0.5);
                else {
                    mechanumDrive(0, 0, 0);
                    goto1 = false;
                }
            }
        }
    }
    public void forwardDropCone() {
        if (motorFrontLeft.getCurrentPosition() < 3200)
            mechanumDrive(1, 0, 0);
        else {
            claw.setPosition(0.5);
            mechanumDrive(0, 0, 0);

            coneDown = true;
        }
    }
    public boolean turn(int angle) {
        if (turning) {
            if (motorFrontLeft.getCurrentPosition() < 1000*(angle/90))
            {
                mechanumDrive(0, 0, 1);
                return false;
            }
            else {
                turning = false;
                resetMotors();
                return true;
            }
        }
        return false;
    }
    public boolean move(double tiles)
    {
        if(moving){
            if (motorFrontLeft.getCurrentPosition() < (int)(2500*tiles)) {
                mechanumDrive(0, 0, 1);
                return false;
            }
            else {
                moving = false;
                resetMotors();
                return true;
            }
        }
        return false;
    }
    public void resetMotors() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mechanumDrive(0, 0, 0);
    }
    public void autonDrive() {
        autonVisionTelemetry();
        /*if (!swivelDone && shouldSwivel) {
                swivel();
        }
        else if (swivelDone && shouldSwivel) {
                mechanumDrive(0, 0, 0);
                resetMotors();
                shouldSwivel = false;
        }
        else if (!coneDown && shouldCone)
            forwardDropCone();
        else if (coneDown && shouldCone) {
            resetMotors();
            shouldCone = false;
        }
        else if (/*!strafeDone !turnFinished /*shouldStrafe) {
            /*strafe();turnNoStrafe();
        }
        else if (/*strafeDoneturnFinished && shouldTurn/*shouldStrafe) {
            resetMotors();
            shouldTurn/*Strafe = false;
        }
//        else if(!goto1 && needs90)
//            turn90();
        */
        if (red) {
            if (tagDetected == 1) {
                if (!movingDone) {
                    moving = true;
                    movingDone = move(1.25);
                } else if (!turningDone) {
                    turning = true;
                    turningDone = turn(90);
                } else if (!movingDone1) {
                    moving = true;
                    movingDone1 = move(1);
                } else
                    auton = false;
            }
            if (tagDetected == 2) {
                if (!movingDone) {
                    moving = true;
                    movingDone = move(1.25);
                } else if (!turningDone) {
                    turning = true;
                    turningDone = turn(90);
                } else if (!movingDone1) {
                    moving = true;
                    movingDone1 = move(1);
                } else if (!turningDone1) {
                    turning = true;
                    turningDone1 = turn(90);
                } else if (!movingDone2) {
                    moving = true;
                    movingDone2 = move(1);
                } else
                    auton = false;
            }
            if (tagDetected == 3) {
                if (!movingDone) {
                    moving = true;
                    movingDone = move(1.25);
                } else if (!turningDone) {
                    turning = true;
                    turningDone = turn(90);
                } else if (!movingDone1) {
                    moving = true;
                    movingDone1 = move(1);
                } else if (!turningDone1) {
                    turning = true;
                    turningDone1 = turn(90);
                } else if (!movingDone2) {
                    moving = true;
                    movingDone2 = move(1);
                } else
                    auton = false;
            }
        }
        else {
            if(!swivelDone)
            {
                swivel = true;
                swivelDone = turn(180);
            }
            if (tagDetected == 1) {
                if (!movingDone) {
                    moving = true;
                    movingDone = move(1.25);
                } else if (!turningDone) {
                    turning = true;
                    turningDone = turn(90);
                } else if (!movingDone1) {
                    moving = true;
                    movingDone1 = move(1);
                } else
                    auton = false;
            }
            if (tagDetected == 2) {
                if (!movingDone) {
                    moving = true;
                    movingDone = move(1.25);
                } else if (!turningDone) {
                    turning = true;
                    turningDone = turn(90);
                } else if (!movingDone1) {
                    moving = true;
                    movingDone1 = move(1);
                } else if (!turningDone1) {
                    turning = true;
                    turningDone1 = turn(90);
                } else if (!movingDone2) {
                    moving = true;
                    movingDone2 = move(1);
                } else
                    auton = false;
            }
            if (tagDetected == 3) {
                if (!movingDone) {
                    moving = true;
                    movingDone = move(1.25);
                } else if (!turningDone) {
                    turning = true;
                    turningDone = turn(90);
                } else if (!movingDone1) {
                    moving = true;
                    movingDone1 = move(1);
                } else if (!turningDone1) {
                    turning = true;
                    turningDone1 = turn(90);
                } else if (!movingDone2) {
                    moving = true;
                    movingDone2 = move(1);
                } else
                    auton = false;
            }
        }
        /*if (/*strafeDone turnFinished&& !shouldTurn/*!shouldStrafe) {
            if (tagDetected == 1) {
//                if (motorFrontLeft.getCurrentPosition() > -1000)
//                    mechanumDrive(-1, 0, 0);
//                else
                    auton = false;
            }
            if (tagDetected == 2) {
                    turning = true;
                    turn90();
                    if (motorFrontLeft.getCurrentPosition() > -2000)
                        mechanumDrive(-1, 0, 0);
                    else
                        auton = false;
            }

            if (tagDetected == 3) {
                    turning = true;
                    turn90();
                    if (motorFrontLeft.getCurrentPosition() > -4500)
                        mechanumDrive(-1, 0, 0);
                    else
                        auton = false;
            }

//             Actually do something useful
            if (tagOfInterest == null) {
                mechanumDrive(0, 0, 0);
                auton = false;
            }
        }*/
    }
    public void tankDrive() {
        powerRight = 0;
        powerLeft = 0;
// tanvi is the bestestestestestest
        if (Math.abs(gamepad1.left_stick_y) > DEADZONE) {
            powerLeft = -gamepad1.left_stick_y;
        }
        if (Math.abs(gamepad1.right_stick_y) > DEADZONE) {
            powerRight = -gamepad1.right_stick_y;
        }
        motorFrontRight.setPower(powerRight);
//        motorFrontLeft.setPower(powerLeft);
//        motorBackRight.setPower(powerRight);
        motorBackLeft.setPower(powerLeft);
    }

    public void mechanumDrive(double forward, double strafe, double turn) {
        double r = Math.hypot(strafe, forward);
        double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
        double rightX = turn;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        motorFrontLeft.setPower(v1*.9);
        motorFrontRight.setPower(v4);
        motorBackLeft.setPower(v3*.9);
        motorBackRight.setPower(v2);
    }

    public void elevatorMove()
    {
        currElevTics = elevator.getCurrentPosition();
        elevator.setPower(1);
//        if(gamepad1.dpad_down) {
//            calibrate = false;
//            calib();
//        }
//        else if (calibrate) {
            if (gamepad1.right_trigger > DEADZONE) {
                if (currElevTics < MAXELEVTICS - 150)
                    elevator.setTargetPosition(currElevTics + 150);
                else
                    elevator.setTargetPosition(MAXELEVTICS);
            }
            if (gamepad1.left_trigger > DEADZONE) {
                if (currElevTics > MINELEVTICS + 150)
                    elevator.setTargetPosition(currElevTics - 150);
                else
                    elevator.setTargetPosition(MINELEVTICS);
            }
            if (gamepad1.y)
                elevator.setTargetPosition(3983);
            if (gamepad1.b)
                elevator.setTargetPosition(2300);
            if (gamepad1.a)
                elevator.setTargetPosition(0);
        currElevTics = elevator.getCurrentPosition();
//        }
    }
    public void clawMove() {
//        telemetry.addData("Claw servo position:", claw.getPosition());
        if (gamepad1.left_bumper)
            claw.setPosition(.5);
        if (gamepad1.right_bumper)
            claw.setPosition(0.2);
    }

    public void calib(){
        elevator.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//        elevator.setDirection(DcMotorEx.Direction.);
        telemetry.addData("elevator calibrating...", elevator.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("elevator position", elevator.getCurrentPosition());
        if(elevator.getCurrent(CurrentUnit.AMPS) < MOTORSTALLVALUE)
        {
            elevator.setPower(-.2);
        }
        else {
            calibrate = true;
            elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }
    public void motorInit(){
        motorFrontLeft = this.hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorBackLeft = this.hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorFrontRight = this.hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackRight = this.hardwareMap.get(DcMotorEx.class, "motorBackRight");
        elevator = this.hardwareMap.get(DcMotorEx.class, "elevator");
        claw = this.hardwareMap.get(Servo.class, "claw");
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.elevator.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void visionInit(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }
    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("\nTag Count ID=%d", tagCount));
    }

    public void aprilTagInitLoop() {
        tagCount = 0;

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                    tagOfInterest = tag;
                    tagFound = true;
                    tagCount++;
                    break;
                }
            }

            if (tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagDetected = tagOfInterest.id;
                tagToTelemetry(tagOfInterest);

            } else {
                tagCount = 0;
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        } else {
            tagCount = 0;
            telemetry.addLine("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                telemetry.addLine("(A tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

    }

    public void autonVisionTelemetry() {
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("Tag was not detected, only dropping off the cone");
            telemetry.update();
        }
    }
}