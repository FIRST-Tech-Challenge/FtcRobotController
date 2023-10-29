package org.firstinspires.ftc.teamcode.Toros;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MainDrive")
@Config
public class Drive2 extends LinearOpMode {

    //PIDF
    private PIDController controller;


    public static double p = 0.03, i = 0, d = -0.0001;
    public static double f = -0.05;
    public static int target = 0;
    private final double ticks_in_degrees = 1440 / 180;

    //Motors
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;
    private DcMotorEx Arm1;

    //Servos
    private Servo Claw1;
    private Servo Claw2;
    private Servo Claw3;


    @Override
    public void runOpMode() throws InterruptedException {
        //Creation for instance of PIDF controller and Telemetry using FTC Dash
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Initializing Hardware in method down below called initHardware();
        initHardware();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //Gamepad controls
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                //Drive variables used in the calculations to run our motors
                double speed = 1;
                double theta = Math.atan2(y, x);
                double power = Math.hypot(x, y);
                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);
                double max = Math.max(Math.abs(sin), Math.abs(cos));

                //Calculations for our drive motors

                double fl = (power * cos / max + turn);
                double fr = (power * sin / max - turn);
                double bl = (power * sin / max + turn);
                double br = (power * cos / max - turn);

                //If statement below is to make sure one motor does not exceed the power limit making it scale down

                if ((power + Math.abs(turn)) > 1) {
                    fl /= power + Math.abs(turn);
                    fr /= power + Math.abs(turn);
                    bl /= power + Math.abs(turn);
                    br /= power + Math.abs(turn);
                }

                telemetry.addData("Target Position", target);

                //Motor Drive
                FrontLeftMotor.setPower(fl * speed);
                FrontRightMotor.setPower(fr * speed);
                BackLeftMotor.setPower(bl * speed);
                BackRightMotor.setPower(br * speed);

                //Servo Control
                Claw1.setPosition((gamepad2.right_stick_y * 180));
                Claw2.setPosition((gamepad2.right_stick_y * 180));
                Claw3.setPosition(gamepad2.left_stick_x * 0.5);

                //Servo Control for Intake and controlling Speed of Motors
                if (gamepad2.left_bumper) {
                    Claw3.setPosition(180);
                } else if (gamepad2.right_bumper) {
                    Claw3.setPosition(0);
                } else if (gamepad1.dpad_down) {
                    speed -= .2;
                } else if (gamepad1.dpad_up) {
                    speed += .2;
                } else if (gamepad1.dpad_left) {
                    speed = 1;
                }

                //ArmControl
                if (gamepad2.y) {
                    pidf(700);
                } else if(gamepad2.x){
                    pidf(1251);
                }
                //Telemetry under iniTelemetry();
                initTelemetry();
            }
        }
    }

    private void initHardware(){
        //Motors
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        Arm1 = hardwareMap.get(DcMotorEx.class, "Arm");
        //Servos
        Claw1 = hardwareMap.get(Servo.class, "Claw1");
        Claw2 = hardwareMap.get(Servo.class, "Claw2");
        Claw3 = hardwareMap.get(Servo.class, "Claw3");
        //Reversal of Motors
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //Zero Power Behaviors
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //More servo stuff
        Claw2.setDirection(Servo.Direction.REVERSE);
        Claw1.setPosition(0);
        Claw2.setPosition(0);
        Claw3.setPosition(0);
    }
    //PIDF control method for controlling the arm
    private void pidf(int targetPOS){
        controller.setPID(p,i,d);
        int armPos = Arm1.getCurrentPosition();
        double pid = controller.calculate(armPos, targetPOS);
        double ff = Math.cos(Math.toRadians(targetPOS / ticks_in_degrees)) * f;

        double powerA = pid + ff;
        Arm1.setPower(powerA);
    }

    private void initTelemetry() {
        //telemetry.addData("Powerfl", String.valueOf(fl));
        //telemetry.addData("Powerfr", String.valueOf(fr));
        //telemetry.addData("Powerbl", String.valueOf(bl));
        //telemetry.addData("Powerbr", String.valueOf(br));
        telemetry.addData("claw angle", Claw1.getPosition() * 180);
        telemetry.addData("claw open or closed", Claw3.getPosition());
        telemetry.addData("Target", target);
        //telemetry.addData("ArmPosition", Arm1.getCurrentPosition());
        telemetry.addData("Pos", Arm1.getCurrentPosition());

        telemetry.update();
    }
}
