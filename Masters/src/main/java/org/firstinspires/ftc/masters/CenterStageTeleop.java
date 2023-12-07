package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name="Center Stage TeleOp", group = "competition")
public class CenterStageTeleop extends LinearOpMode {
    double y = 0;
    double x = 0;
    double rx = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;

    DcMotor gpSlideRight = null;
    DcMotor gpSlideLeft = null;
    DcMotor backSlides = null;
    DcMotor hangingMotor = null;

    Servo planeLaunch;
    Servo planeRaise;
    Servo clawServo;
    Servo clawArm;
    Servo clawAngle;
    Servo cameraTurning;
    Servo outtakeHook;
    Servo outtakeRotation;
    Servo outtakeMovementRight;
    Servo outtakeMovementLeft;

    double[] servoPos = {.5,.5,.5,.5,.5,.5,.5,.5,.5,.5};
    int target = 0;

    int clawClosed = 0;

    private enum DriveMode {
        NORMAL,
        PIXEL_SCORE,
        END_GAME
    }
    private DriveMode driveMode = DriveMode.NORMAL;
    @Override
    public void runOpMode() {
        /*
            Controls
            --------
            Gamepad2

            Gamepad1

            --------
Drive with sticks intake is the front
Four bar position 1-5 auto stack pixels
Slides position 1-11 placement on backboard
Normal mode - pixel grabbing mode - entered with left stick down
D-pad controls extendo
D-pad up - extendo fully extended with four bar in position 4
D-pad right - extendo out 2/3? with four bar in position 1
D-pad down - extendo out 1/3? with four bar in position 1
D-pad left - extendo fully in and four bar in position 1
Buttons
A - claw opens and closes
B - transfers and moves to slide pos 1
X - auto aligns and switches to pixel scoring mode
Y - press once drop one pixel, hold for drop both pixels, once both are placed outtake goes back into transfer
LB - four bar down (presets)
RB - four bar up (presets)
LT - slides down (presets)
RT - slides up (presets)

Pixel scoring mode - entered with x while in normal
Only moves right and left no forward/ backwards
LT - slides down (presets)
RT - slides up (presets)
Y - press once drop one pixel, hold for drop both pixels

Endgame mode - entered with right stick down
Normal driving
X - Auto aligns on April tag for shooter, raise shooter, shoot
LB - Hang down
RB - Hang up
        */
        leftFrontMotor = hardwareMap.dcMotor.get("frontLeft");
        rightFrontMotor = hardwareMap.dcMotor.get("frontRight");
        leftRearMotor = hardwareMap.dcMotor.get("backLeft");
        rightRearMotor = hardwareMap.dcMotor.get("backRight");

        hangingMotor = hardwareMap.dcMotor.get("hangingMotor");

        gpSlideLeft = hardwareMap.dcMotor.get("gpSlideLeft");
        gpSlideRight = hardwareMap.dcMotor.get("gpSlideRight");
        backSlides = hardwareMap.dcMotor.get("backSlides");

        planeLaunch = hardwareMap.servo.get("planeLaunch");
        planeRaise = hardwareMap.servo.get("planeRaise");
        clawServo = hardwareMap.servo.get("clawServo");
        clawArm = hardwareMap.servo.get("clawArm");
        clawAngle = hardwareMap.servo.get("clawAngle");
        cameraTurning = hardwareMap.servo.get("cameraTurning");
        outtakeHook = hardwareMap.servo.get("outtakeHook");
        outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        outtakeMovementRight = hardwareMap.servo.get("outtakeMovementRight");
        outtakeMovementLeft = hardwareMap.servo.get("outtakeMovementLeft");

        // Set the drive motor direction:
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            telemetry.addData("linear slide encoder",  + linearSlideMotor.getCurrentPosition());

            telemetry.addData("left y", gamepad1.left_stick_y);
            telemetry.addData("left x", gamepad1.left_stick_x);
            telemetry.addData("right x", gamepad1.right_stick_x);

            switch (driveMode)  {
                case NORMAL:
                    /*
X - auto aligns and switches to pixel scoring mode
Y - press once drop one pixel, hold for drop both pixels, once both are placed outtake goes back into transfer
LB - four bar down (presets for stack positions)
RB - four bar up (presets)
LT - slides down (presets)
RT - slides up (presets)
                    */
                    if (gamepad1.dpad_up){
                        //gp slides full extension
                        //v4b preset 4 pixel high
                        //claw open
                    }
                    if (gamepad1.dpad_right){
                        //gp slides 2/3rds extension
                        //v4b preset 1 pixel high
                        //claw open
                    }
                    if (gamepad1.dpad_down){
                        //gp slides 1/3rds extension
                        //v4b preset 1 pixel high
                        //claw open
                    }
                    if (gamepad1.dpad_left){
                        //gp slides fully in
                        //v4b preset 4 pixel high
                        //claw open
                    }
                    if (gamepad1.a && clawClosed == 1) {
                        //claw false
                        //claw open
                        sleep(300);
                    } else if (gamepad1.a) {
                        //claw true
                        //claw close
                        sleep(300);
                    }
                    if (gamepad1.b) {
                        //slides in
                        //v4b to deposit
                        //claw angle to deposit
                        //when not busy, open claw
                        //right intake
                    }
                    if (gamepad1.x) {
                        // Solinexi's april tag alignment
                        driveMode = DriveMode.PIXEL_SCORE;
                    }
                    if (gamepad1.left_stick_button) {
                        driveMode = DriveMode.PIXEL_SCORE;
                        sleep(300);
                    }
                    break;
                case PIXEL_SCORE:
                    break;
                case END_GAME:
                    break;
            }
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.2) {
                y = 0;
            }
            if (Math.abs(x) < 0.2) {
                x = 0;
            }

            double leftFrontPower = y + x + rx;
            double leftRearPower = y -  x + rx;
            double rightFrontPower = y -  x - rx;
            double rightRearPower = y + x - rx;

            if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1) {

                double max;
                max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
                max = Math.max(max, Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(rightRearPower));

                leftFrontPower /= max;
                leftRearPower /= max;
                rightFrontPower /= max;
                rightRearPower /= max;
            }

            leftFrontMotor.setPower(leftFrontPower);
            leftRearMotor.setPower(leftRearPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightRearMotor.setPower(rightRearPower);

            if (gamepad1.a && servoPos[target]< 1) {
                servoPos[target]+=.005;
            } else if (gamepad1.b && servoPos[target] > 0) {
                servoPos[target]-=.005;
            }

            if (gamepad1.x) {
                target++;
                if (target == 10){
                    target = 0;
                }
                sleep(300);
            } else if (gamepad1.y) {
                target--;
                if (target == -1){
                    target = 9;
                }
                sleep(300);
            }

            if(gamepad1.right_bumper){
                hangingMotor.setPower(.3);
            } else if (gamepad1.left_bumper) {
                hangingMotor.setPower(-.3);
            } else {
                hangingMotor.setPower(0);
            }

            if (gamepad1.right_trigger > .3) {
                hangingMotor.setPower(.1);
            } else if (gamepad1.left_trigger > .3) {
                hangingMotor.setPower(-.1);
            } else {
                backSlides.setPower(0);
            }

            planeLaunch.setPosition(servoPos[0]);
            planeRaise.setPosition(servoPos[1]);
            clawServo.setPosition(servoPos[2]);
            clawArm.setPosition(servoPos[3]);
            clawAngle.setPosition(servoPos[4]);
            cameraTurning.setPosition(servoPos[5]);
            outtakeHook.setPosition(servoPos[6]);
            outtakeRotation.setPosition(servoPos[7]);
            outtakeMovementRight.setPosition(servoPos[8]);
            outtakeMovementLeft.setPosition(servoPos[9]);

            telemetry.addData("planeLaunch",servoPos[0]);
            telemetry.addData("planeRaise",servoPos[1]);
            telemetry.addData("clawServo",servoPos[2]);
            telemetry.addData("clawArm",servoPos[3]);
            telemetry.addData("clawAngle",servoPos[4]);
            telemetry.addData("cameraTurning",servoPos[5]);
            telemetry.addData("outtakeHook",servoPos[6]);
            telemetry.addData("outtakeRotation",servoPos[7]);
            telemetry.addData("outtakeMovementRight",servoPos[8]);
            telemetry.addData("outtakeMovementLeft",servoPos[9]);
            telemetry.addData("hangingMotor",hangingMotor.getCurrentPosition());
            telemetry.addData("backSlides",backSlides.getCurrentPosition());

            telemetry.update();
        }
    }
}