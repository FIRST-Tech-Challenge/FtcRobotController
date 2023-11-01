/*
package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.Bot.detections;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.AprilTagsPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp
public class MainTeleOp extends LinearOpMode {


    private GamepadEx gp1, gp2;

    Bot bot= Bot.getInstance(this);
    private boolean isAutomatic;
    private double driveSpeed=1;


    @Override
    public void runOpMode() throws InterruptedException {
        gp2 = new GamepadEx(gamepad2);
        gp1 = new GamepadEx(gamepad1);

        //Camera Stuff
        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        bot.camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        bot.aprilTagsPipeline = new AprilTagsPipeline(detections.tagsize, detections.fx, detections.fy, detections.cx, detections.cy);


        bot.camera.setPipeline(bot.aprilTagsPipeline);
        bot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                bot.camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

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
                telemetry.addData("selected automatic driving mode",isAutomatic);
            }

            //Finite state machine enabled
            if(isAutomatic) {
                //noodle intake
                if(gp2.wasJustPressed(GamepadKeys.Button.Y)) {
                    bot.noodles.Intake();
                    while(!bot.box.getIsFull()) {
                        bot.box.checkBeam();
                        bot.noodles.Intake();
                    }
                    if(bot.box.getIsFull()) {
                        bot.noodles.stop();
                        bot.prepForOuttake();
                    }
                }

                //slide movement (automatic stages)
                if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    bot.outtake(3,false);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    bot.outtake(2,false);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    bot.outtake(4, false);
                } else if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    bot.outtake(1, false);
                }

                //keeping second pixel deposit manual for reasons
                if(gp2.wasJustPressed(GamepadKeys.Button.A) && bot.box.getNumPixelsDeposited()==1) {
                    bot.box.depositSecondPixel();
                    bot.resetOuttake();
                }

                //drone + sus code
                if(gp2.wasJustPressed(GamepadKeys.Button.B)) {
                //  Bot.suspension.hang();
                    bot.drone.shoot();
                }
            }

            //Disabled finite state machine
            if(!isAutomatic){
                //drone movement
                if(gp2.wasJustPressed(GamepadKeys.Button.X)){
                    bot.drone.shoot();
                    bot.drone.reset();
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
                    bot.outtakeBox();
                }
                if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                    bot.outtakeBox();
                }

                //intake
                if(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                    double power = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    while(gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0.1){
                        bot.intake(power);
                        power = gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                    }
                    bot.intake(0);
                }


                //slides
                if(gp2.getLeftY()!=0){
                    double raw = gp2.getLeftY();
                    bot.outtakeSlides(raw*1800);
                    //the max value for the joystick will be equal to 1, so the max value for runTo in slides will be 1800
                }

                //fourbar
                if(gp2.getRightY()>0){
                    bot.outtakeFourbar(gp2.getRightY());
                }
                if(gp2.getRightY()<0){
                    bot.outtakeFourbar(gp2.getLeftY());
                }
            }
            //constantly checking for drive inputs
            drive();
        }
    }

    private void drive() {
        if (gp1.wasJustReleased(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
        }
     //   driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
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


 */