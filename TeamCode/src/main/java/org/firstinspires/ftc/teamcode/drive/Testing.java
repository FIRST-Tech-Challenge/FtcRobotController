package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SlidingArmVD;
import org.firstinspires.ftc.teamcode.aa.testingsuites.genericConfigManager;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.ConfigMan;

import java.util.HashMap;
import java.util.List;

@SuppressWarnings("OpMode")
@TeleOp(group = "drive")
public class Testing extends LinearOpMode {
    public int schemeNumber;
    // Gamepad controls
    double rx;
    double ry;
    boolean buttonA;
    boolean buttonB;
    boolean buttonX;
    boolean buttonY;
    boolean dpadUp;
    boolean dpadDown;
    boolean dpadLeft;
    boolean dpadRight;
    double rt;
    double lt;
    double lx;
    double ly;
    boolean lb;
    boolean rb;
    boolean start;
    boolean back;

    
    Telemetry telemetries;

    genericConfigManager configMan = new genericConfigManager();

    @SuppressWarnings("Servos assemble")
    List servos;
    DcMotorEx armMotor1;
    DcMotorEx armMotor2;
    DcMotorEx motor1;
    DcMotorEx motor2;

    Servo drone;

    public void updateGamepad() {
        rx = gamepad1.right_stick_x;
        ry = gamepad1.right_stick_y;
        buttonA = gamepad1.a;
        buttonB = gamepad1.b;
        buttonX = gamepad1.x;
        buttonY = gamepad1.y;
        dpadUp = gamepad1.dpad_up;
        dpadDown = gamepad1.dpad_down;
        dpadRight = gamepad1.dpad_right;
        dpadLeft = gamepad1.dpad_left;
        rt = gamepad1.left_trigger;
        lt = gamepad1.right_trigger;

        lx = gamepad1.left_stick_x;
        ly = gamepad1.left_stick_y;

        lb = gamepad1.left_bumper;
        rb = gamepad1.right_bumper;

        back = gamepad1.back;
        start = gamepad1.start;

    }

    @Override
    public void runOpMode() {
        int mode = 0;
        boolean started = false;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servos = hardwareMap.getAll(Servo.class);
        telemetries = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Intiate Hardware
        armMotor1 = drive.hardware.armMotor1;
        armMotor2 = drive.hardware.armMotor2;

        drone = drive.hardware.droneServo;

        // Begin User Input
        telemetries.addLine("Running Op Mode \n Note, used lb adn rb at teh same time to start ConfigMan, to change hardcoded values in the software.");
        telemetries.update();

        waitForStart();

        telemetries.addLine("Please select a mode");
        telemetries.addLine("Mode 0: Mecanum Drive Test \n Mode 1: Arm Slider Test \n Mode 2: Drone Launcher Test \n Mode 3: Claw Test");
        telemetries.update();
        telemetries.addLine("Mode 0: A \n Mode 1: B \n Mode 2: X \n Mode 3: Y");
        telemetries.update();
        while (opModeIsActive() && !started) {

            updateGamepad();
            if (buttonA) {
                mode = 0; started=true;
            }
            if (buttonB) {
                mode = 1; started=true;
            }
            if (buttonX) {
                mode = 2; started=true;
            }
            if (buttonY) {
                mode = 3; started=true;
            }
        }
        switch (mode) {
            case 0:

                break;
            case 1:
                ArmSlidingTestSuite();
                break;
            case 2:
                droneTestSuite();
                break;
            case 3:
                double valueServo1;
                double valueServo2;
                telemetries.addLine("Detect (B) or Set (X)");
                telemetries.update();
                while (started) {
                    updateGamepad();
                    if (buttonX) {
                        while (started) {
                            updateGamepad();
                            valueServo2 = Range.clip(ly, 0, 0.25);
                            valueServo1 = Range.clip(lx, 0, 0.25);
                            drive.hardware.servo1.setPosition(valueServo1);
                            drive.hardware.servo2.setPosition(valueServo2);
                            if(back) {
                                started = false;
                            }
                            checkContrConfRun("case3");
                        }
                    }
                    if (buttonB) {
                        while (started) {
                            valueServo1 = drive.hardware.servo1.getPosition();
                            valueServo2 = drive.hardware.servo2.getPosition();
                            telemetries.addLine("Servo Position 1: " + valueServo1);
                            telemetries.addLine("Servo Position 2: " + valueServo2);
                            telemetries.update();
                            if(back) {
                                started = false;
                            }
                            checkContrConfRun("case3");
                        }
                    }
                    if(back) {
                        started = false;
                    }
                }
                break;
        }

    }

    public HashMap ReturnConfigMan() {
        return new HashMap();
    }
    private void ArmSlidingTestSuite() {
        boolean active = true;
        String schemeName = "";
        try {
            schemeName = reserveScheme();
        } catch (Exception e) {
            active = false;
        }
        HashMap<String, Integer> currentConfig = configMan.getScheme(schemeName);
        SlidingArmVD arm1 = new SlidingArmVD( "1344", "device", currentConfig, 1, armMotor1);
        SlidingArmVD arm2 = new SlidingArmVD("1344", "device", currentConfig, 1, armMotor2);
        updateGamepad();
        while (active) {
            telemetries.addLine( "Please select the desired sub mode: \n A) Detect \n B) Drive  ");
            if (buttonA) {
                // Detect
                while (active) {
                    telemetries.addLine("Sliding Arms' States: ");
                    telemetries.addData("Length 1", arm1.getLengthPosition());
                    telemetries.addData("Rotational Position 1", arm1.getRotationalPosition());
                    telemetries.addData("Length 2", arm2.getLengthPosition());
                    telemetries.addData("Rotational Position 2", arm2.getRotationalPosition());
                    telemetries.update();
                    updateGamepad();
                    if (back) {
                        break;
                    }
                    checkContrConfRun("armTest");

                }
                active = true;
            } 
            if (buttonB) {
                // Drive
                while (active) {
                    telemetries.addLine("Sliding Arms' States: ");
                    telemetries.addData("Length 1", arm1.getLengthPosition());
                    telemetries.addData("Rotational Position 1", arm1.getRotationalPosition());
                    telemetries.addData("Length 2", arm2.getLengthPosition());
                    telemetries.addData("Rotational Position 2", arm2.getRotationalPosition());
                    telemetries.update();
                    updateGamepad();
                    if (buttonA) {
                        arm1.SetPosition(0);
                    }
                    if (buttonB) {
                        arm1.SetPosition(30);
                    }
                    if (buttonX) {
                        arm1.SetPosition(60);
                    }
                    if (buttonY) {
                        arm1.SetPosition(90);
                    }
                    if (dpadDown) {
                        arm2.SetPosition(0);
                    }
                    if (dpadRight) {
                        arm2.SetPosition(30);
                    }
                    if (dpadLeft) {
                        arm2.SetPosition(60);
                    }
                    if (dpadUp) {
                        arm2.SetPosition(90);
                    }
                    if(back) {
                        break;
                    }
                    checkContrConfRun("armTest");
                }
            }
            if (back) {
                active = false;
            }
            checkContrConfRun("armTest");
            updateGamepad();
        }
        return;
    }
    private String reserveScheme() throws Exception{
        if (schemeNumber == 0) {
            schemeNumber++;
            return "scheme1";
        } else if (schemeNumber == 1) {
            schemeNumber++;
            return "scheme2";
        }else if (schemeNumber == 2) {
            schemeNumber++;
            return "scheme3";
        }else if (schemeNumber == 3) {
            schemeNumber++;
            return "scheme4";
        }else if (schemeNumber == 4) {
            schemeNumber++;
            return "scheme5";
        }else if (schemeNumber == 5) {
            schemeNumber++;
            return "scheme6";
        }else if (schemeNumber == 0) {
            throw new Exception("no extra schemes present");
        }
        return "";
    }
    private void checkContrConfRun(String env) {
        if (lb && rb) {
            runningConfigManager(env);
        }
    }
    private void runningConfigManager(String env) {
        telemetries.addLine("Welcome to the configurator. ");
    }
    private void droneTestSuite() {
        while (opModeIsActive()) {
            if (buttonA) {
                drone.setDirection(Servo.Direction.REVERSE);
                drone.setPosition(0.5);
            }
        }

    }
}
