package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.team12397.v2.RobotHardware;
import org.firstinspires.ftc.team12397.v2.cameraSoftware.ColorVisionSubsystem;
import org.firstinspires.ftc.team12397.v2.cameraSoftware.util.AngleServoController;

@TeleOp(name="FieldCentric", group="Robot")

public class FieldCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;
        double extenderInches;

        final double rotationBottomPosition = robot.ROTATION_MAXIMUM; // bot is maximum because it would go the farthest forward
        final double rotationTopPosition = robot.ROTATION_MINIMUM; // top is minimum because it would go (-) ticks
        final double rotationSmidgePosition = 360/robot.ROTATE_SLIDE_TICKS_PER_DEGREE; // peak height reaches 12 inches off ground

        double rotationDegrees = 0;

        Gamepad luisL = gamepad1;
        Gamepad alexH = gamepad2;

        robot.init();

        // vision inits must go after robot.init to prevent camera & servo from returning null
        ColorVisionSubsystem vision = new ColorVisionSubsystem(robot.camera);
        AngleServoController servo  = new AngleServoController(robot.clawYaw, telemetry);

        FtcDashboard.getInstance().startCameraStream(vision.getPortal(), 0);

        waitForStart();


        while (opModeIsActive()) {

            drive = -luisL.left_stick_y;
            strafe = luisL.left_stick_x;
            turn = luisL.right_stick_x;

            if (alexH.a){ // bottom
                rotationDegrees = rotationBottomPosition;
                robot.rotateMotor.setVelocity(1250);
            } else if (alexH.y){ // up
                rotationDegrees = rotationTopPosition;
                robot.rotateMotor.setVelocity(1250);
            } else if (alexH.x){ // slightly elevated
                rotationDegrees = rotationSmidgePosition;
            }

            // if alex moves his left stick up/down more than a hundredth of maximum movement...
            if (Math.round(Math.abs(alexH.left_stick_y*10)) != 0){
                // set target position to previous distance +/- fudge amount
                extenderInches = robot.slideExtender.getCurrentPosition()/robot.EXTEND_SLIDE_TICKS_PER_INCH;
                extenderInches = extenderInches + -alexH.left_stick_y*4;
                // fudge amount is 3.5 inches: 1/5 of maximum reach.
            } else {
                // if alex DOES NOT move his left stick, stop arm at current position.
                extenderInches = robot.slideExtender.getCurrentPosition()/robot.EXTEND_SLIDE_TICKS_PER_INCH;
            }

            // action lines (lines of code that actually move the robot)
            robot.driveFieldCentric(drive, strafe, turn);
            robot.RotateSlides(rotationDegrees);
            robot.setExtenderPosition(extenderInches);


            if (alexH.dpad_up){
                robot.setServoPosition(0, 0.1);
            } else if (alexH.dpad_down){
                robot.setServoPosition(0, 0.375);
            }

            if (alexH.dpad_left){
                robot.setServoPosition(1, 0);
            } else if ( alexH.dpad_right){
                robot.setServoPosition(1, 0.685);
            }

            if (alexH.left_bumper){
                // if alex presses left bumper again, go to middle position
                if (robot.clawPitch.getPosition() == 0.05){
                    robot.setServoPosition(2, 0.5);
                } else { // else, go to the default left bumper position
                    robot.setServoPosition(2, 0.05);
                }
            } else if (alexH.right_bumper){
                // if alex presses right bumper again, go to middle position
                if (robot.clawPitch.getPosition() == 1){
                    robot.setServoPosition(2, 1);
                } else { // else, go to the default right bumper position
                    robot.setServoPosition(2, 1);
                }
            }

            // tester control
            if (luisL.a){
                vision.update();

                if (vision.hasTarget()) {
                    servo.update(vision.getAngleErrorToVertical());
                }

                telemetry.addData("Target?", vision.hasTarget());
                telemetry.addData("Angle°",  "%.1f", vision.getAngle());
                telemetry.addData("Area",    "%.0f", vision.getArea());
            }

            telemetry.addData(String.valueOf(robot.rotateMotor.getCurrentPosition()), " rotation encoder ticks");
            telemetry.update();

        }
    }
}
