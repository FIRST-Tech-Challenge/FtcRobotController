package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp
public class AnimosityAndMortification extends Movable {
    //variables
    static double tgtPower = 0;
    static double tgtPower2 = 0;
    static boolean stop = false;
    static private boolean servoOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
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

            // FRW is the only right moving wheel :(`
            if(gamepad1.left_stick_y > 0.3 || gamepad1.left_stick_y < -0.3) {
                //forward and backwards
                FLW.setPower(-tgtPower);
                BLW.setPower(-tgtPower);
                FRW.setPower(-tgtPower);
                BRW.setPower(tgtPower);
            } else if (gamepad1.left_stick_x > 0.3 || gamepad1.left_stick_x < -0.3) {
                FLW.setPower(tgtPower2);
                BLW.setPower(-tgtPower2);
                FRW.setPower(-tgtPower2);
                BRW.setPower(-tgtPower2);
            } else if (gamepad1.left_bumper) {
                FLW.setPower(-0.5);
                BLW.setPower(-0.5);
                FRW.setPower(0.5);
                BRW.setPower(-0.5);
            } else if (gamepad1.right_bumper) {
                FLW.setPower(0.5);
                BLW.setPower(0.5);
                FRW.setPower(-0.5);
                BRW.setPower(0.5);
            } else if (gamepad1.left_trigger > .75 && !stop) {
                new Thread(() -> {
                    try {
                        FLW.setPower(-0.5);
                        BLW.setPower(-0.5);
                        FRW.setPower(0.5);
                        BRW.setPower(-0.5);
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

                        FLW.setPower(0.5);
                        BLW.setPower(0.5);
                        FRW.setPower(-0.5);
                        BRW.setPower(0.5);
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
                FRScissorLift.setPower(1);
                FLScissorLift.setPower(1);
                BRScissorLift.setPower(-1);
                BLScissorLift.setPower(-1);
            }else if(gamepad1.right_stick_y < -0.5) {
                FRScissorLift.setPower(-1);
                FLScissorLift.setPower(-1);
                BRScissorLift.setPower(1);
                BLScissorLift.setPower(1);
            }else {
                disableScissorPower();
            }

//            if (gamepad1.a) {
//                if (servoOpen) {
//                    ServoR.setPosition(1);
//                    ServoL.setPosition(1);
//                    servoOpen = false;
//                } else {
//                    ServoR.setPosition(0);
//                    ServoL.setPosition(0);
//                    servoOpen = true;
//                }
//            }

            if (gamepad1.a) {
                powerScissorLift(0, "up");
            } else if (gamepad1.b) {
                powerScissorLift(0, "down");
            }  else {
                disableScissorPower();
            }
            updatePhoneConsole();
        }
    }

    public static void disablePowerWheels() {
        FLW.setPower(0);
        BLW.setPower(0);
        FRW.setPower(0);
        BRW.setPower(0);}

    public void updatePhoneConsole() {
        telemetry.addData("Status","Running");
        telemetry.addData("Target Power:", tgtPower);
        telemetry.addData("FLW Power:", FLW.getPower());
        telemetry.addData("BLW Power:", BLW.getPower());
        telemetry.addData("FRW Power:", FRW.getPower());
        telemetry.addData("BRW Power:", BRW.getPower());
        telemetry.addData("FRScissorLift Power:", FRScissorLift.getPower());
        telemetry.addData("BRScissorLift Power:", BRScissorLift.getPower());
        telemetry.addData("FLScissorLift Power:", FLScissorLift.getPower());
        telemetry.addData("BLScissorLift Power:", BLScissorLift.getPower());
        telemetry.addData("Left Stick X:", tgtPower2);
        telemetry.addData("Left Stick Y:", tgtPower);

        telemetry.update();
    }
}