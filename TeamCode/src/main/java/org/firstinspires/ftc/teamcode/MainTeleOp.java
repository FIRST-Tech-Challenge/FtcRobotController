package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;


@TeleOp
public class MainTeleOp extends LinearOpMode {

    private DcMotorEx motor_fr;
    private DcMotorEx motor_fl;
    private DcMotorEx motor_br;
    private DcMotorEx motor_bl;
    private IMU imu;
    private GamepadEx gp1, gp2;
    Bot bot;
    private boolean isAutomatic;
    private boolean firstPixelIsDesposited;


    private double driveSpeed=1;

    @Override
    public void runOpMode() throws InterruptedException {
        motor_fr = hardwareMap.get(DcMotorEx.class, "frontRight");
        motor_fl = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motor_br = hardwareMap.get(DcMotorEx.class, "backRight");
        motor_bl = hardwareMap.get(DcMotorEx.class, "backLeft");
        imu = hardwareMap.get(IMU.class, "IMU?");
        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);


        telemetry.addData("teleOp is ", "initialized");

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("teleOp is ", "running");
            telemetry.update();
            gp1.readButtons();
            gp2.readButtons();

            if(gp2.wasJustPressed(GamepadKeys.Button.START)){
                isAutomatic=!isAutomatic;
            }

            if(isAutomatic){

                //noodle intake
                if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                        Bot.noodles.Intake();
                        /*  while(!holdFull){
                                Bot.noodles.Intake();
                          }
                          if(holdFull){
                            Bot.noodles.stop()
                            Bot.fourbar.outtake();
                          }
                          */
                    //when break beam sensor detects hold if full, stop intaking and bring fourbar to outtake position
                }

                //slide movement (automatic stages)
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    Bot.slides.runTo(3);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    Bot.slides.runTo(2);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    Bot.slides.runTo(4);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    Bot.slides.runTo(1);
                }


                //Box deposits
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && !firstPixelIsDesposited){
                //    Bot.box.depositeFirstPixel();
                    firstPixelIsDesposited= true;
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && firstPixelIsDesposited) {
                 //   Bot.box.depositSecondPixel();
                    Bot.box.resetBox();
                  //  Bot.slides.storage();
                    Bot.fourbar.storage();
                    firstPixelIsDesposited = false;
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                //   Bot.suspension.hang();
                    Bot.drone.shoot();

                }

            }



            if(!isAutomatic){

                //drone movement
                if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                    Bot.drone.shoot();
                    Bot.drone.reset();
                }

                //suspension hang
                if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                  //  Bot.suspension.hang();
                }

                //Box movement
                if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                    Bot.box.resetBox();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && !firstPixelIsDesposited){
                 //   Bot.box.depositFirstPixel();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && firstPixelIsDesposited){
                    //Bot.box.depositSecondPixel();
                }

                //intake movement
                if(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                    double power = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    while(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                        Bot.noodles.intake(power);
                        power= gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    }
                    Bot.noodles.stop();
                }

                //JOYSTICK CODE??? FOR SLIDES AND FOURBAR

                //slides code
                if(gp2.getLeftY()!=0){
                    Bot.slides.runTo(gp2.getLeftY());
                }

                //fourbar code
                if(gp2.getRightY()>0){
                    Bot.fourbar.runManualOuttake(gp2.getLeftY());
                }
                if(gp2.getRightY()<0){
                    Bot.fourbar.runManualStorage(gp2.getLeftY());
                }

            }
        }
    }


    private void drive() {
        if (gp1.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            bot.resetIMU();
        }
        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);

        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);
        bot.driveRobotCentric(driveVector.getX() * driveSpeed,
                driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed / 1.7
        );
    }
}
