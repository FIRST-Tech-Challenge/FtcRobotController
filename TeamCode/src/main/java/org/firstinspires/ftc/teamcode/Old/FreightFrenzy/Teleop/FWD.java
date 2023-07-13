package org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFGamepad;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.FWDRobot;
/*
FOUR WHEEL DRIVE ROBOT CONTROLS
UPDATED 9/2/2022 10:04 PM - Harry

GAMEPAD 1:
Y - open claw
X - close claw
A - raise arm
B - lower arm
Left Stick Y axis - forward/backwards
Right Stick X axis - left/right
Right Stick Y axis - Extend/retract tape
LB - toggle disc intake off
RB - toggle disc intake on
DPAD_Right - move tape Y axis
DPAD_Left - move tape X axis

GAMEPAD 2: NONE
 */

@Config
@Disabled
@TeleOp(name = "FWD")
public class FWD extends LinearOpMode {
    // motors & servos
    private DcMotorEx wobbleArmGoal;
    private Servo wobbleGoalGrabbyer;
    private CRServo tapeExtend;
    private Servo tapeHeight;
    private Servo tapeMove;
    private DcMotorEx discIntake;

    private PIDController controller; // pid controller creation
    public static double p = 0, i = 0, d = 0; // initialize pidf variables
    public static double f = 0;

    public static int target = 0; // default initialization of target position

    private final double ticks_in_degree = 1680 / 360; // constant ratio of ticks to degrees

    @Override
    public void runOpMode() throws InterruptedException {
        FWDRobot robit = new FWDRobot(this, true); // declaration of FWDRobot constructor, pass in this opmode
        RFGamepad gp = new RFGamepad();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // ftc dashboard telemetry functionality
        // hardwaremapping most of the motor/servos
        discIntake = (DcMotorEx) hardwareMap.dcMotor.get("discIntake");
        discIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleArmGoal = (DcMotorEx) hardwareMap.dcMotor.get("wobbleArmGoal");
        wobbleGoalGrabbyer = hardwareMap.servo.get("wobbleGoalGrab");
        tapeExtend = hardwareMap.crservo.get("tapeExtend");
        tapeHeight = hardwareMap.servo.get("tapeHeight");
        tapeMove = hardwareMap.servo.get("tapeMove");
        controller = new PIDController(p, i, d); // pid controller declaration

        wobbleArmGoal.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArmGoal.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        wobbleArmGoal.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        wobbleGoalGrabbyer.setPosition(0); //set servo to open position as default
        f = 0; p = 0; d = 0; i = 0; // reset PIDF coeffs when init so you don't carry over previous run's values
        waitForStart();
        // used to make tape meausure need less buttons
        int onetwo = 0;
        int twoone = 0;

        resetRuntime();
        while (opModeIsActive() && getRuntime() < 90 ) { // opmode active check & set time limit

            if (gamepad1.a) { // arm raised position
                target = 460;
            } else if (gamepad1.b) { // arm lowered position
                target = 45;
            }

            controller.setPID(p, i, d); // pid controller
            int armPos = wobbleArmGoal.getCurrentPosition(); // self explanatory
            double pid = controller.calculate(armPos, target); // calculate only pid power needed given current arm position & target position
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f; // maths
            double power = pid + ff; // adding ff coeff to the pidf, hence the "f"
            wobbleArmGoal.setPower(power);
            if (gamepad1.y) {
                wobbleGoalGrabbyer.setPosition(0);
                // tuned values
                f = 0;
                p = 0.00025;
            }
            if (gamepad1.x) {
                wobbleGoalGrabbyer.setPosition(1);
                // tuned values
                p = 0.0004;
            }
            if (gamepad1.right_bumper) {
                discIntake.setPower(0.5);
            }
            else if (gamepad1.left_bumper) {
                discIntake.setPower(0);
            }
            tapeExtend.setPower(gamepad1.right_stick_y);
            if (gamepad1.dpad_left) {
                onetwo += 1;
                tapeMove.setPosition(onetwo % 2);
            }
            if (gamepad1.dpad_right) {
                twoone += 1;
                tapeHeight.setPosition(twoone % 2);
            }

            gp.readGamepad(gamepad1.y, "gamepad1_y", "Open Claw");
            gp.readGamepad(gamepad1.x, "gamepad1_x", "Close Claw");
            gp.readGamepad(gamepad1.a, "gamepad1_a", "Riase Arm");
            gp.readGamepad(gamepad1.b, "gamepad1_b", "Lower Arm");
            gp.readGamepad(gamepad1.left_stick_y, "gamepad1_left_stick_y", "Forward/Backwards");
            gp.readGamepad(gamepad1.right_stick_x, "gamepad1_right_stick_x", "Left/Right");
            gp.readGamepad(gamepad1.right_stick_y, "gamepad1_right_stick_y", "Extend/Retract Tape");
            gp.readGamepad(gamepad1.left_bumper, "gamepad1_left_bumper", "Disc intake Off");
            gp.readGamepad(gamepad1.right_bumper, "gamepad1_right_bumper", "Disc Intake On");
            gp.readGamepad(gamepad1.dpad_left, "gamepad1_dpad_left", "Tape Up/Down");
            gp.readGamepad(gamepad1.dpad_right, "gamepad1_dpad_right", "Tape Left/Right");

            robit.motorRightBack.setPower(gamepad1.left_stick_y * 0.3 + gamepad1.right_stick_x * 0.4);
            robit.motorLeftBack.setPower(gamepad1.left_stick_y * 0.3 - gamepad1.right_stick_x * 0.4);
            robit.motorRightFront.setPower(gamepad1.left_stick_y * 0.3 + gamepad1.right_stick_x * 0.4);
            robit.motorLeftFront.setPower(gamepad1.left_stick_y * 0.3 - gamepad1.right_stick_x * 0.4);
            telemetry.addData("Left Back Motor Pos", robit.motorLeftBack.getCurrentPosition());
            telemetry.addData("Left Front Motor Pos", robit.motorLeftFront.getCurrentPosition());
            telemetry.addData("Right Back Motor Pos", robit.motorRightBack.getCurrentPosition());
            telemetry.addData("Right Front Motor Pos", robit.motorRightFront.getCurrentPosition());
            telemetry.addData("wobble goal arm pos", wobbleArmGoal.getCurrentPosition());
            telemetry.addData("velo", wobbleArmGoal.getVelocity());
            telemetry.addData("pos", wobbleArmGoal.getCurrentPosition());
            telemetry.addData("target", target);
            telemetry.update();
            sleep(40);


        }
    }
}