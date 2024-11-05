package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp
public class HatredAndDepression2 extends LinearOpMode {
    //variables
    static double tgtPower = 0;
    static double tgtPower2 = 0;
    static boolean stop = false;
    static private DcMotor FLW;
    static private DcMotor BLW;
    static private DcMotor FRW;
    static private DcMotor BRW;
    static private DcMotor ArmL;
    static private DcMotor ArmR;

    @Override
    public void runOpMode() throws InterruptedException {
        FLW = hardwareMap.get(DcMotor.class, "FLW");
        BLW = hardwareMap.get(DcMotor.class, "BLW");
        BRW = hardwareMap.get(DcMotor.class, "BRW");
        FRW = hardwareMap.get(DcMotor.class, "FRW");
        ArmL = hardwareMap.get(DcMotor.class, "ArmL");
        ArmR = hardwareMap.get(DcMotor.class, "ArmR");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            tgtPower = this.gamepad1.left_stick_y;
            tgtPower2 = this.gamepad1.left_stick_x;

            if(gamepad1.left_stick_y > 0.3 || gamepad1.left_stick_y < -0.3) {
                //forward and backwards
                FLW.setPower(tgtPower);
                //BLW.setPower(tgtPower);
                //FRW.setPower(-tgtPower);
                //BRW.setPower(-tgtPower);
            } else if (gamepad1.left_stick_x > 0.3 || gamepad1.left_stick_x < -0.3) {
                FLW.setPower(-tgtPower2);
                BLW.setPower(-tgtPower2);
                FRW.setPower(-tgtPower2);
                BRW.setPower(-tgtPower2);
            } else if (gamepad1.left_bumper) {
                FLW.setPower(0.5);
                BLW.setPower(-0.5);
                FRW.setPower(0.5);
                BRW.setPower(-0.5);
            } else if (gamepad1.right_bumper) {
                FLW.setPower(-0.5);
                BLW.setPower(0.5);
                FRW.setPower(-0.5);
                BRW.setPower(0.5);
            } else if (gamepad1.left_trigger > .75 && !stop) {
                new Thread(() -> {
                    try {
                        FLW.setPower(.5);
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

            } else if (gamepad1.right_trigger > .75 && !stop) {

                new Thread(() -> {
                    try {
                        FLW.setPower(-.5);
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
            } else if(gamepad1.dpad_up) {
                FLW.setPower(1);
                // wrong, should go forward
            } else if(gamepad1.dpad_down){
                FRW.setPower(1);
                // right, just change 1 to -1
            } else if(gamepad1.dpad_left){
                BLW.setPower(1);
                // right
            } else if(gamepad1.dpad_right){
                BRW.setPower(1);
                // right
            }else {
                disablePowerWheels();
            }

            //arm
            if(gamepad1.right_stick_y > 0.5){
                ArmL.setPower(.5);
                ArmR.setPower(-.5);
            }else if(gamepad1.right_stick_y < -0.5) {
                ArmL.setPower(-.5);
                ArmR.setPower(.5);
            }else {
                disableArmMotors();
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
        ArmL.setPower(0);
        ArmR.setPower(0);
    }

    public void updatePhoneConsole() {
        telemetry.addData("Status","Running");
        telemetry.addData("Target Power:", tgtPower);
        telemetry.addData("FLW Power:", FLW.getPower());
        telemetry.addData("BLW Power:", BLW.getPower());
        telemetry.addData("FRW Power:", FRW.getPower());
        telemetry.addData("BRW Power:", BRW.getPower());
        telemetry.addData("ArmL Power:", ArmL.getPower());
        telemetry.addData("ArmR Power:", ArmR.getPower());
        telemetry.addData("Left Stick X:", tgtPower2);
        telemetry.addData("Left Stick Y:", tgtPower);
        telemetry.update();
    }
}