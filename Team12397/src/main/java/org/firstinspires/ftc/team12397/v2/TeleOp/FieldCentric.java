package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.team12397.v2.RobotHardware;

@TeleOp(name="FieldCentric", group="Robot")

public class FieldCentric extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive;
        double strafe;
        double turn;
        double extenderInches;

        double rotationDegrees = 0;

        Gamepad luisL = gamepad1;
        Gamepad alexH = gamepad2;

        waitForStart();
        robot.init();

        while (opModeIsActive()) {

            drive = -luisL.left_stick_y;
            strafe = luisL.left_stick_x;
            turn = luisL.right_stick_x;

            if(alexH.y){
                rotationDegrees = -20;
            } else if (alexH.a){
                rotationDegrees = 20;
            }

            // if alex moves his left stick up/down more than a hundreth of maximum movement...
            if (Math.round(Math.abs(alexH.left_stick_y*10)) != 0){
                // set target position to previous distance +/- fudge amount
                extenderInches = robot.slideExtender.getCurrentPosition()/robot.EXTEND_SLIDE_TICKS_PER_INCH;
                extenderInches = extenderInches + -alexH.left_stick_y*3;
                // fudge amount is 3.5 inches: 1/5 of maximum reach.
            } else {
                // if alex DOES NOT move his left stick, stop arm at current position.
                extenderInches = robot.slideExtender.getCurrentPosition()/robot.EXTEND_SLIDE_TICKS_PER_INCH;
            }

            // action lines (lines of code that actually move the robot)
            robot.driveFieldCentric(drive, strafe, turn);
            robot.RotateSlides(rotationDegrees);
            robot.setExtenderPosition(extenderInches);


            if(alexH.dpad_up){
                robot.setServoPosition(0, 0.1);
            } else if ( alexH.dpad_down){
                robot.setServoPosition(0, 0.375);
            }

            if(alexH.dpad_left){
                robot.setServoPosition(1, 0);
            } else if ( alexH.dpad_right){
                robot.setServoPosition(1, 0.685);
            }

            if(alexH.left_bumper){
                robot.setServoPosition(2, 0.05);
            } else if (alexH.right_bumper){
                robot.setServoPosition(2, 1);
            }

        }
    }
}
