package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@Disabled
@TeleOp(name="Mecanum TeleOp", group="TeleOp")
public class MecanumTeleOp extends LinearOpMode {

    // Motors & Sensors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private SparkFunOTOS odometry;

    // Gamepad2 Toggles
    private boolean gamepad2YClicking = false;
    private boolean gamepad2BClicking = false;
    private boolean gamepad2XClicking = false;
    private boolean gamepad2AClicking = false;

    // Servo Vars
    private double wristReference = 0.5;
    private double gripperReference = 0.5;
    private double lolclock = 0.01;

    // Funzies
    private static final boolean allowAnthonyExtras = false;

    // Servos
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;

    // Subsystem
    private ArmSubSystem armSystem;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware map
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        DcMotor spindle = hardwareMap.get(DcMotor.class, "spindle");
        odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
        DcMotor cap = hardwareMap.get(DcMotor.class, "cap");
        LSLower = hardwareMap.get(ServoImplEx.class, "LSLower");
        LSTop = hardwareMap.get(ServoImplEx.class, "LSTop");
        RevTouchSensor lswitch = hardwareMap.get(RevTouchSensor.class, "Lswitch");


        // Define Servo range
        LSLower.setPwmEnable();
        LSTop.setPwmEnable();
        LSLower.scaleRange(0, 1);
        LSTop.scaleRange(0, 1);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Breaking mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make sure odo is ready
        odometry.begin();

        // Initialize the ArmSubSystem
        armSystem = new ArmSubSystem(armPose.REST, cap, spindle, lswitch, LSTop, LSLower, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if (lolclock > 0.05) {
                lolclock = lolclock - 0.001;
            } else {
                lolclock = lolclock + 0.001;
            }
            // Get joystick values
            double y;
            double x;
            double rx;

            if (gamepad1.left_bumper) {
                y = (gamepad1.left_stick_y * 0.25) + (Math.pow(gamepad1.left_stick_y, 3) * 0.2);
                x = (-gamepad1.left_stick_x * 0.25) + (Math.pow(-gamepad1.left_stick_x, 3) * 0.2);
                rx = -gamepad1.right_stick_x * 0.25;
            } else {
                y = Math.pow(gamepad1.left_stick_y, 3) * (1 - gamepad1.left_trigger);
                x = Math.pow(-gamepad1.left_stick_x, 3) * (1 - gamepad1.left_trigger);
                rx = -gamepad1.right_stick_x * (1 - gamepad1.left_trigger);
            }

            // Update References with ArmSystem

            if (gamepad2.a && !gamepad2AClicking) {
                gamepad2AClicking = true;
                armSystem.goToRest(); // If A Clicked Go to Rest
            }
            if (!gamepad2.a) {
                gamepad2AClicking = false;
            }

            if (gamepad2.b && !gamepad2BClicking) {
                gamepad2AClicking = true;
                armSystem.cycleBasketTop(); // If B Clicked Go to Basket
            }
            if (!gamepad2.b) {
                gamepad2BClicking = false;
            }

            if (gamepad2.x && !gamepad2XClicking) {
                gamepad2XClicking = true;
                armSystem.cycleSubmersible(); // If X Clicked Go to Submersible
            }
            if (!gamepad2.x) {
                gamepad2XClicking = false;
            }

            if (gamepad2.y && !gamepad2YClicking) {
                gamepad2YClicking = true;
                armSystem.cycleChamberTop(); // If Y Clicked Go to Top Chamber
            }
            if (!gamepad2.y) {
                gamepad2YClicking = false;
            }
            if (allowAnthonyExtras) {
                if (gamepad1.a && !gamepad2AClicking) {
                    gamepad2AClicking = true;
                    armSystem.goToRest(); // If A Clicked Go to Rest
                }
                if (!gamepad1.a) {
                    gamepad2AClicking = false;
                }

                if (gamepad1.b && !gamepad2BClicking) {
                    gamepad2AClicking = true;
                    armSystem.cycleBasketTop(); // If B Clicked Go to Basket
                }
                if (!gamepad1.b) {
                    gamepad2BClicking = false;
                }

                if (gamepad1.x && !gamepad2XClicking) {
                    gamepad2XClicking = true;
                    armSystem.cycleSubmersible(); // If X Clicked Go to Submersible
                }
                if (!gamepad1.x) {
                    gamepad2XClicking = false;
                }

                if (gamepad1.y && !gamepad2YClicking) {
                    gamepad2YClicking = true;
                    armSystem.cycleChamberTop(); // If Y Clicked Go to Top Chamber
                }
                if (!gamepad1.y) {
                    gamepad2YClicking = false;
                }
            }

            // Set power to motors
            frontLeftMotor.setPower(y + x + rx);
            backLeftMotor.setPower(y - x + rx);
            frontRightMotor.setPower(y - x - rx);
            backRightMotor.setPower(y + x - rx);

            // Arm System Controls

            armSystem.periodicUpdate(new TelemetryPacket());

            // Wrist and Gripper Values
            if (gamepad1.left_trigger > 0.5) {
                wristReference = 1;
            }
            if (gamepad1.right_trigger > 0.5) {
                wristReference = 0.3;
            }
            if (gamepad2.dpad_up) {
                gripperReference = 0.75;
            }
            if (gamepad2.dpad_down) {
                gripperReference = 0.15;
            }
            if (gamepad2.left_bumper) {
                armSystem.enterZeroPos();
            }

            LSLower.setPosition(wristReference - lolclock);
            LSTop.setPosition(gripperReference - lolclock);

            // Telemetry for debugging
            SparkFunOTOS.Pose2D pos = odometry.getPosition();
            telemetry.addData("OdoX", pos.x);
            telemetry.addData("OdoY", pos.y);
            telemetry.addData("Angle", pos.h);
            telemetry.addData("Gripper Position", LSTop.getPosition());
            telemetry.addData("Wrist Position", LSLower.getPosition());
            telemetry.addData("Extendo Reference", armSystem.getExtendoReference());
            telemetry.addData("Capstan Reference", armSystem.getCapstanReference());
            telemetry.update();
        }
    }
}
