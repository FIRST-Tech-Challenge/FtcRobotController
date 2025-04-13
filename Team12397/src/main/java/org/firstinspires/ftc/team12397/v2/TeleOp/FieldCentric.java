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
                rotationDegrees = 0;
            } else if (alexH.a){
                rotationDegrees = 90;
            }


            // if alex moves his left stick up/down more than a hundreth of maximum movement...
            if (Math.round(-alexH.left_stick_y*100) != 0){
                // set target position to previous distance +/- fudge amount
                extenderInches = robot.slideExtender.getCurrentPosition();
                extenderInches = extenderInches + alexH.left_stick_y*1.5;
                // fudge amount is 3.5 inches: 1/5 of maximum reach.
            } else {
                // if alex DOES NOT move his left stick, stop arm at current position.
                extenderInches = robot.slideExtender.getCurrentPosition();
            }

            // action lines (lines of code that actually move the robot)
            robot.driveFieldCentric(drive, strafe, turn);
            robot.RotateSlides(rotationDegrees);
            robot.setExtenderPosition(extenderInches);

            sleep(20);
        }
    }

}
