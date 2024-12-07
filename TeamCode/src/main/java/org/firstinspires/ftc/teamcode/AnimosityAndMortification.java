package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp
public class AnimosityAndMortification extends LinearOpMode {
    //variables
    static double tgtPower = 0;
    static double tgtPower2 = 0;
    static boolean stop = false;
    static private DcMotor FLW;
    static private DcMotor BLW;
    static private DcMotor FRW;
    static private DcMotor BRW;
    static private DcMotor ScissorLiftR;
    static private DcMotor ScissorLiftL;
    static private Servo ServoR;
    static private Servo ServoL;
    static private CRServo ArmRaise;
    static private boolean servoOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");
        FRW = hardwareMap.get(DcMotor.class, "FRW");
        ScissorLiftR = hardwareMap.get(DcMotor.class, "ScissorLiftR"); // change config if needed
        ScissorLiftL = hardwareMap.get(DcMotor.class, "ScissorLiftL");
        ServoR = hardwareMap.get(Servo.class, "ServoR");
        ServoL = hardwareMap.get(Servo.class, "ServoL");
        ArmRaise = hardwareMap.get(CRServo.class, "ArmRaise");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();

            tgtPower = this.gamepad1.left_stick_y/1.4;
            tgtPower2 = this.gamepad1.left_stick_x/1.4;

            // FLW is the only right moving wheel :(`
            if(gamepad1.left_stick_y > 0.3 || gamepad1.left_stick_y < -0.3) {
                //forward and backwards
                FLW.setPower(tgtPower);
                BLW.setPower(-tgtPower);
                FRW.setPower(tgtPower);
                BRW.setPower(tgtPower);
            } else if (gamepad1.left_stick_x > 0.3 || gamepad1.left_stick_x < -0.3) {
                FLW.setPower(-tgtPower2);
                BLW.setPower(tgtPower2);
                FRW.setPower(tgtPower2);
                BRW.setPower(tgtPower2);
            } else if (gamepad1.left_bumper) {
                FLW.setPower(0.5);
                BLW.setPower(0.5);
                FRW.setPower(-0.5);
                BRW.setPower(0.5);
            } else if (gamepad1.right_bumper) {
                FLW.setPower(-0.5);
                BLW.setPower(-0.5);
                FRW.setPower(0.5);
                BRW.setPower(-0.5);
            } else if (gamepad1.left_trigger > .75 && !stop) {
                new Thread(() -> {
                    try {
                        FLW.setPower(.5);
                        BLW.setPower(.5);
                        FRW.setPower(-.5);
                        BRW.setPower(.5);
                        Thread.sleep(210);
                        disablePowerWheels();

                        stop = true;
                        Thread.sleep(1000);
                        stop = false;

                    } catch(Exception e) {
                    }
                }).start();

            } else if (gamepad1.right_trigger > .75 && !stop) {
                new Thread(() -> {
                    try {
                        FLW.setPower(-.5);
                        BLW.setPower(-.5);
                        FRW.setPower(.5);
                        BRW.setPower(-.5);
                        Thread.sleep(210);
                        disablePowerWheels();

                        stop = true;
                        Thread.sleep(1000);
                        stop = false;
                    } catch(Exception e) {
                    }
                }).start();
            } else {
                disablePowerWheels();
            }

            // scissor lift
            // enter real values once you get to test
            if(gamepad1.right_stick_y > 0.5){
                ScissorLiftL.setPower(0.5);
                ScissorLiftR.setPower(-0.5);
            }else if(gamepad1.right_stick_y < -0.5) {
                ScissorLiftL.setPower(-0.5);
                ScissorLiftR.setPower(0.5);
            }else {
                disableArmMotors();
            }

            if (gamepad1.a) {
                if (servoOpen) {
                    ServoR.setPosition(1); // change these values later
                    ServoL.setPosition(1);
                    servoOpen = false;
                } else {
                    ServoR.setPosition(0);
                    ServoL.setPosition(0);
                    servoOpen = true;
                }
            } else {
                disableScissorMovement();
            }
            updatePhoneConsole();
        }
    }

    public static void disablePowerWheels() {
        FLW.setPower(0);
        BLW.setPower(0);
        FRW.setPower(0);
        BRW.setPower(0);
    }

    public static void disableArmMotors()   {
        ScissorLiftL.setPower(0);
        ScissorLiftR.setPower(0);
    }

    public static void disableScissorMovement()   {
        ArmRaise.setPower(0);
    }

    public void updatePhoneConsole() {
        telemetry.addData("Status","Running");
        telemetry.addData("Target Power:", tgtPower);
        telemetry.addData("FLW Power:", FLW.getPower());
        telemetry.addData("BLW Power:", BLW.getPower());
        telemetry.addData("FRW Power:", FRW.getPower());
        telemetry.addData("BRW Power:", BRW.getPower());
        telemetry.addData("ScissorLiftL Power:", ScissorLiftL.getPower());
        telemetry.addData("ScissorLiftR Power:", ScissorLiftR.getPower());
        telemetry.addData("Left Stick X:", tgtPower2);
        telemetry.addData("Left Stick Y:", tgtPower);
        telemetry.addData("Arm Raise Power Value:", ArmRaise.getPower());
        telemetry.addData("Arm Raise Direction:", ArmRaise.getDirection());

        telemetry.update();
    }
}