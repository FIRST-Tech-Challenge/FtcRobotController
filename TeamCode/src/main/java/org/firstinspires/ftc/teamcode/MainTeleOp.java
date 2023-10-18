package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;




@TeleOp
public class MainTeleOp extends LinearOpMode {

    private DcMotorEx motor_fr;
    private DcMotorEx motor_fl;
    private DcMotorEx motor_br;
    private DcMotorEx motor_bl;
    private IMU imu;
    private GamepadEx gp1, gp2;
    private DistanceSensor distanceSensor;
    private double distanceFromObject;
    private double perfectDistance= 5;

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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        imu = hardwareMap.get(IMU.class, "IMU?");
        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);

        distanceFromObject = distanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("teleOp is ", "initialized");

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("teleOp is ", "running");
            telemetry.update();
            gp1.readButtons();
            gp2.readButtons();
            telemetry.update();


            if(gp2.wasJustPressed(GamepadKeys.Button.START)) {
                isAutomatic = !isAutomatic;
            }

            //Finite state machine enabled
            if(isAutomatic) {
                //noodle intake
                if(gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    Bot.noodles.Intake();
                    while(!bot.box.getIsFull()) {
                        bot.box.boxIsFull();
                        Bot.noodles.Intake();
                    }
                    if(bot.box.getIsFull()) {
                        Bot.noodles.stop();
                        bot.prepForOuttake();
                    }
                }

                //slide movement (automatic stages)
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.outtake(3,distanceSensor,false);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.outtake(2,distanceSensor,false);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.outtake(4,distanceSensor,false);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.outtake(1,distanceSensor,false);
                }

                //keeping second pixel deposit manual for reasons
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && Bot.box.getNumPixelsDeposited()==1) {
                    bot.box.depositSecondPixel();
                    Bot.resetOuttake();
                }

                //drone + sus code
                if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                //  Bot.suspension.hang();
                    Bot.drone.shoot();
                }
            }

            //Disabled finite state machine
            if(!isAutomatic){
                //drone movement
                if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                    Bot.drone.shoot();
                    Bot.drone.reset();
                }

                //suspension
                if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                  //Bot.suspension.hang();
                }

                //Box movement
                if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                    bot.box.resetBox();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                    bot.outtake();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                    bot.outtake();
                }

                //intake
                if(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                    double power = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    while(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                        Bot.noodles.intake(power);
                        power= gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    }
                    Bot.noodles.stop();
                }


                //slides
                if(gp2.getLeftY()!=0){
                    Bot.slides.runTo(gp2.getLeftY());
                    //the left y on the y joystick will be at max 1. We need to change this
                }

                //fourbar
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
