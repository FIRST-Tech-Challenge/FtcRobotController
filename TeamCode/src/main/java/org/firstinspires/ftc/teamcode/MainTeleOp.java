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
        motor_fr = hardwareMap.get(DcMotorEx.class, "fr");
        motor_fl = hardwareMap.get(DcMotorEx.class, "fl");
        motor_br = hardwareMap.get(DcMotorEx.class, "br");
        motor_bl = hardwareMap.get(DcMotorEx.class, "bl");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        imu = hardwareMap.get(IMU.class, "IMU?");
        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);

        distanceFromObject = distanceSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("teleOp is ", "initialized");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("teleOp is ", "running");
            telemetry.update();
            gp1.readButtons();
            gp2.readButtons();
            telemetry.update();


            if(gp2.wasJustPressed(GamepadKeys.Button.START)) {
                isAutomatic = !isAutomatic;
            }


//Finite State Machine Organization => two systems: Automatic and Manual (can toggle between modes using the start button

            //automatic state machine
            if(isAutomatic) {

                //noodle intake (incorporates break beam sensor)
                // fourbar immediately goes to outtake position once Box if full
                if(gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    Bot.noodles.Intake();
                    while(!bot.box.getIsFull()) {
                        bot.box.boxIsFull();
                        Bot.noodles.Intake();
                    }
                    if(bot.box.getIsFull()) {
                        Bot.noodles.stop();
                        Bot.fourbar.outtake();
                    }
                }

                //slide movement (automatic stages)
                //driver inputs stage level via DPad => slides use motion profiling to reach position
                //deposit first pixel as soon as slides reach specific stage
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.distanceTuning(distanceSensor);
                    Bot.slides.runTo(3);
                    bot.box.depositFirstPixel();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.distanceTuning(distanceSensor);
                    Bot.slides.runTo(2);
                    bot.box.depositFirstPixel();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.distanceTuning(distanceSensor);
                    Bot.slides.runTo(4);
                    bot.box.depositFirstPixel();
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.distanceTuning(distanceSensor);
                    Bot.slides.runTo(1);
                    bot.box.depositFirstPixel();
                }

                //can desposit second pixel by clicking A
                // not automatic so driver can move robot horizontally to deposit this pixel in a specific location
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && Bot.box.getNumPixelsDeposited()==1) {
                    bot.box.depositSecondPixel();
                    Bot.resetOuttake();
                }

                // as soon as suspension is achieved, shoot drone
                if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                //   Bot.suspension.hang();
                    Bot.drone.shoot();
                }

            }


            //manual state machine
            if(!isAutomatic){

                //drone movement
                if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                    Bot.drone.shoot();
                    Bot.drone.reset();
                }

                //suspension hang
                if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                  //Bot.suspension.hang();
                }

                //Box movement
                if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                    bot.box.resetBox();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && !firstPixelIsDesposited){
                    bot.box.depositFirstPixel();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && firstPixelIsDesposited){
                    bot.box.depositSecondPixel();
                }

                //intake movement (intake faster when pressing trigger harder)
                if(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                    double power = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    while(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                        Bot.noodles.intake(power);
                        power= gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    }
                    Bot.noodles.stop();
                }


                //slides code (manually set power of slides via left joystick y value)
                if(gp2.getLeftY()!=0){
                    Bot.slides.runTo(gp2.getLeftY());
                }

                //fourbar code (manually set power of slides via right joystick y value)
                if(gp2.getRightY()>0){
                    Bot.fourbar.runManualOuttake(gp2.getLeftY());
                }
                if(gp2.getRightY()<0){
                    Bot.fourbar.runManualStorage(gp2.getLeftY());
                }
            }
            drive();
        }
    }


    //drivetrain strafing
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
