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


    public static double p = 0.03, i = 0.0022, d = 0.001;
    public static double f = -0.05;
    public static int target = -100;
   private final double ticks_in_degrees = 1440 / 180;

    //Motors
    private DcMotor FrontLeftMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackRightMotor;
    private DcMotor Railgun;
    private DcMotorEx Arm1;

    //Servos
    private Servo Claw1;
    private Servo Claw2;
    private Servo Claw3;

    double speed = 100;

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
                boolean PIDF = true;

                //Drive variables used in the calculations to run our motors
                boolean launcher = gamepad1.y;
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

                //telemetry.addData("Target Position", target);

                //Motor Drive
                FrontLeftMotor.setPower(fl * (speed/100));
                FrontRightMotor.setPower(fr * (speed/100));
                BackLeftMotor.setPower(bl * (speed/100));
                BackRightMotor.setPower(br * (speed/100));

                //Servo Control
                Claw1.setPosition((gamepad2.right_stick_y * 180));
                Claw2.setPosition((gamepad2.right_stick_y * 180));
                Claw3.setPosition(gamepad2.left_stick_x * 0.5);
                Arm1.setPower(gamepad2.left_stick_y);
                //Servo Control for Intake and controlling Speed of Motors
                if (gamepad2.left_bumper) {
                    Claw3.setPosition(180);
                } else if (gamepad2.right_bumper) {
                    Claw3.setPosition(0);
                } else if (gamepad1.dpad_down) {
                    speed -= 2;
                } else if (gamepad1.dpad_up) {
                    speed += 2;
                } else if (gamepad1.dpad_left) {
                    speed = 100;
                } if (speed > 100){
                    speed = 100;
                } else if (speed < 2){
                    speed = 2;
                }
                if (target > -2100){
                    f =  0.05;
                } else if (target < -2100) {
                    f = -0.05;
                }
                double powerA = 0;
                int armPos = Arm1.getCurrentPosition();

                controller.setPID(p, i, d);
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;
                powerA = pid + ff;

                if(gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0){
                    powerA = gamepad2.left_stick_y / 0.5;
                    target = armPos;
                }


                Arm1.setPower(powerA);



                //ArmControl
                if (gamepad2.y) {
                    target = -2850;
                } else if (gamepad2.x) {
                    target = -1900;
                } else if (gamepad2.b){
                    target = -550;
                } else if(gamepad2.a){
                    target =-3250;
                }

                if(launcher == true){
                    Railgun.setPower(1);
                }

//                if (gamepad2.left_stick_y <= 1.0 && gamepad2.left_stick_y != 0.0|| gamepad2.left_stick_y >= -1.0 && gamepad2.left_stick_y != 0 ){
//                    target = (int) (target + 10 * gamepad2.left_stick_y);
//                    if (target  > -75) {
//                        target = -75;
//                    } else if (target <-4300) {
//                        target = -4300;
//                    }
//                }


//
//                if(gamepad2.y){
//                    Arm1.setPower(1.0);
//                } else if (gamepad2.b) {
//                    Arm1.setPower(0);
//                }
                //Telemetry under iniTelemetry();
                telemetry.addData("Speed", speed);
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
        Railgun = hardwareMap.get(DcMotor.class,"Railgun");
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


    private void initTelemetry() {

        telemetry.addData("claw angle", Claw1.getPosition() * 180);
        telemetry.addData("claw open or closed", Claw3.getPosition());
        //
        //
        //
        telemetry.addData("Target", target);
        telemetry.addData("ArmPosition", Arm1.getCurrentPosition());
        telemetry.addData("Pos", Arm1.getCurrentPosition());
        telemetry.addData("Speed", speed);
        telemetry.update();
    }
}