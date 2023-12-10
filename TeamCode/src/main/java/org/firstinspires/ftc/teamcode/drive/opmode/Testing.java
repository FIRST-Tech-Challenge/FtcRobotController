package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.aa.testingsuites.ElectricalTesting;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.virtualdevices.SlidingArmVD;
import org.firstinspires.ftc.teamcode.aa.testingsuites.genericConfigManager;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.ConfigMan;

import java.util.HashMap;
import java.util.List;

class Testing extends LinearOpMode {

    boolean lb;
    boolean rb;
    double lt;
    double rt;
    double lx;
    double rx;
    double ly;
    double ry;
    boolean dpadUp;
    boolean dpadDown;
    boolean dpadLeft;
    boolean dpadRight;
    boolean buttonA;
    boolean buttonB;
    boolean buttonX;
    boolean buttonY;
    boolean start;
    boolean back;

    genericConfigManager configMan = new genericConfigManager();
    protected int schemeNumber = 0;

    Telemetry telemetries;

    DcMotorEx armMotor1;
    DcMotorEx armMotor2;

    DcMotorEx motor0;
    DcMotorEx motor1;
    DcMotorEx motor2;
    DcMotorEx motor3;
    DcMotorEx motor4;
    DcMotorEx motor5;
    DcMotorEx motor6;
    DcMotorEx motor7;

    DcMotorEx[] motors;

    Servo servo0;
    Servo servo1;
    Servo servo2;
    Servo servo3;
    Servo servo4;
    Servo servo5;
    Servo servo6;
    Servo servo7;
    Servo servo8;
    Servo servo9;
    Servo servo10;
    Servo servo11;

    Servo[] servos;


    // TODO: Set Up Digital Channel Support and allow it to change forms, for different connected devices;
    // TODO: Set Up Analog  Channel Support and allow it to change forms, for different connected devices;

    Servo drone;

    private void updateGamepad() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        int mode = 0;
        boolean started = false;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetries = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        HashMap<String, Object> centralScheme;
        try {
            centralScheme = configMan.getScheme(genericConfigManager.getSchemeName(configMan.reserveScheme()));
        } catch (Exception e) {
            stop();
            throw new RuntimeException(e);
        }
        // Intiate Hardware
        armMotor1 = drive.hardware.armMotor1;
        armMotor2 = drive.hardware.armMotor2;
        try {
            motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        } catch (IllegalArgumentException e) {
            centralScheme.put("motor0Usability", 0);
        }
        try {
            motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        } catch (IllegalArgumentException e) {
            centralScheme.put("motor1Usability", 0);
        }
        try {
            motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        } catch (IllegalArgumentException e) {
            centralScheme.put("motor2Usability", 0);
        }
        try {
            motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        } catch (IllegalArgumentException e) {
            centralScheme.put("motor3Usability", 0);
        }
        try {
            motor4 = hardwareMap.get(DcMotorEx.class, "exMotor0");
        } catch (IllegalArgumentException e) {
            centralScheme.put("motor4Usability", 0);
        }
        try {
            motor5 = hardwareMap.get(DcMotorEx.class, "exMotor1");
        } catch (IllegalArgumentException e) {
            centralScheme.put("motor5Usability", 0);
        }
        try {
            motor6 = hardwareMap.get(DcMotorEx.class, "exMotor2");
        } catch (IllegalArgumentException e) {
            centralScheme.put("motor6Usability", 0);
        }
        try {
            motor7 = hardwareMap.get(DcMotorEx.class, "exMotor3");
        } catch (IllegalArgumentException e) {
            centralScheme.put("motor7Usability", 0);
        }

        try {
            servo0 = hardwareMap.get(Servo.class, "servo0");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo0Usability", 0);
        }
        try {
            servo1 = hardwareMap.get(Servo.class, "servo1");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo1Usability", 0);
        }
        try {
            servo2 = hardwareMap.get(Servo.class, "servo2");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo2Usability", 0);
        }
        try {
            servo3 = hardwareMap.get(Servo.class, "servo3");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo3Usability", 0);
        }
        try {
            servo4 = hardwareMap.get(Servo.class, "servo4");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo4Usability", 0);
        }
        try {
            servo5 = hardwareMap.get(Servo.class, "servo5");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo5Usability", 0);
        }
        try {
            servo6 = hardwareMap.get(Servo.class, "exServo0");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo6Usability", 0);
        }
        try {
            servo7 = hardwareMap.get(Servo.class, "exServo1");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo7Usability", 0);
        }
        try {
            servo8 = hardwareMap.get(Servo.class, "exServo2");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo8Usability", 0);
        }
        try {
            servo9 = hardwareMap.get(Servo.class, "exServo3");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo9Usability", 0);
        }
        try {
            servo10 = hardwareMap.get(Servo.class, "exServo4");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo10Usability", 0);
        }
        try {
            servo11 = hardwareMap.get(Servo.class, "exServo5");
        } catch (IllegalArgumentException e) {
            centralScheme.put("servo11Usability", 0);
        }

        drone = drive.hardware.droneServo;
        // Collect Servos & Motors
        if ((int) centralScheme.get("motor0Usability") == 1) {
            motors[0] = motor0;
        }
        if ((int) centralScheme.get("motor1Usability") == 1) {
            motors[1] = motor1;
        }
        if ((int) centralScheme.get("motor2Usability") == 1) {
            motors[2] = motor2;
        }
        if ((int) centralScheme.get("motor3Usability") == 1) {
            motors[3] = motor3;
        }
        if ((int) centralScheme.get("motor4Usability") == 1) {
            motors[4] = motor4;
        }
        if ((int) centralScheme.get("motor5Usability") == 1) {
            motors[5] = motor5;
        }
        if ((int) centralScheme.get("motor6Usability") == 1) {
            motors[6] = motor6;
        }
        if ((int) centralScheme.get("motor7Usability") == 1) {
            motors[7] = motor7;
        }

        if ((int) centralScheme.get("servo0Usability") == 1) {
            servos[0] = servo0;
        }
        if ((int) centralScheme.get("servo1Usability") == 1) {
            servos[1] = servo1;
        }
        if ((int) centralScheme.get("servo2Usability") == 1) {
            servos[2] = servo2;
        }
        if ((int) centralScheme.get("servo3Usability") == 1) {
            servos[3] = servo3;
        }
        if ((int) centralScheme.get("servo4Usability") == 1) {
            servos[4] = servo4;
        }
        if ((int) centralScheme.get("servo5Usability") == 1) {
            servos[5] = servo5;
        }
        if ((int) centralScheme.get("servo6Usability") == 1) {
            servos[6] = servo6;
        }
        if ((int) centralScheme.get("servo7Usability") == 1) {
            servos[7] = servo7;
        }
        if ((int) centralScheme.get("servo8Usability") == 1) {
            servos[8] = servo8;
        }
        if ((int) centralScheme.get("servo9Usability") == 1) {
            servos[9] = servo9;
        }
        if ((int) centralScheme.get("servo10Usability") == 1) {
            servos[10] = servo10;
        }
        if ((int) centralScheme.get("servo11Usability") == 1) {
            servos[11] = servo11;
        }

        // Begin User Input
        telemetries.addLine("Running Op Mode \n Note, used lb adn rb at teh same time to start ConfigMan, to change hardcoded values in the software.");
        telemetries.update();

        waitForStart();

        telemetries.addLine("Please select a mode");
        telemetries.addLine("Mode 0: Mecanum Drive Test \n Mode 1: Arm Slider Test \n Mode 2: Drone Launcher Test \n Mode 3: Claw Test");
        telemetries.addLine("Mode 0: A \n Mode 1: B \n Mode 2: X \n Mode 3: Y");
        telemetries.update();
        while (opModeIsActive() && !started) {
            updateGamepad();
            if (buttonA) {
                mode = 0;
                started = true;
            }
            if (buttonB) {
                mode = 1;
                started = true;
            }
            if (buttonX) {
                mode = 2;
                started = true;
            }
            if (buttonY) {
                mode = 3;
                started = true;
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
                            if (back) {
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
                            if (back) {
                                started = false;
                            }
                            checkContrConfRun("case3");
                        }
                    }
                    if (back) {
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
        HashMap<String, Object> currentConfig = configMan.getScheme(schemeName);
        SlidingArmVD arm1 = new SlidingArmVD("1344", "device", currentConfig, 1, armMotor1);
        SlidingArmVD arm2 = new SlidingArmVD("1344", "device", currentConfig, 1, armMotor2);
        updateGamepad();
        while (active) {
            telemetries.addLine("Please select the desired sub mode: \n A) Detect \n B) Drive  ");
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
                    if (back) {
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
    private String reserveScheme() throws Exception {
        if (schemeNumber == 0) {
            schemeNumber++;
            return "scheme1";
        } else if (schemeNumber == 1) {
            schemeNumber++;
            return "scheme2";
        } else if (schemeNumber == 2) {
            schemeNumber++;
            return "scheme3";
        } else if (schemeNumber == 3) {
            schemeNumber++;
            return "scheme4";
        } else if (schemeNumber == 4) {
            schemeNumber++;
            return "scheme5";
        } else if (schemeNumber == 5) {
            schemeNumber++;
            return "scheme6";
        } else if (schemeNumber == 0) {
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

    private void electricalTesting() throws Exception {
        String schemeName = "";
        schemeName = genericConfigManager.getSchemeName(configMan.reserveScheme());
        ElectricalTesting el = new ElectricalTesting(schemeName, telemetries);
        el.runOpMode();
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
