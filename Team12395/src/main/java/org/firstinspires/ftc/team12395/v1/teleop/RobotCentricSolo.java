package org.firstinspires.ftc.team12395.v1.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.team12395.v1.RobotHardware;


@TeleOp(name="Robot Centric (Solo)", group="Robot")

public class RobotCentricSolo extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        double slide = 0;
        double hslide = 0;
        double vertical = 0.5;
        double horizontalOffset = 0;
        double extendOffset = 1;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            strafe = -gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobotCentric(drive, strafe, turn);

            // Use gamepad left Bumper to open and close the intake claw

            double inClawOffset = 0;
            if (gamepad1.left_bumper) {
                inClawOffset = 1;
            } else {
                inClawOffset = 0;
            }

            robot.setInClawPosition(inClawOffset);

            //Use right bumper to open and close outtake claw

            double outClawOffset = 0;

            if (gamepad1.right_bumper) {
                outClawOffset = 1;
            } else {
                outClawOffset = 0;
            }

            robot.setOutClawPosition(outClawOffset);

            // Move both servos to new position.  Use RobotHardware class

            // Use gamepad buttons to move arm up (Y) and down (A)
            // Use the MOTOR constants defined in RobotHardware class.
            if (gamepad1.y) {
                vertical = 1;
            } else if (gamepad1.a) {
                vertical = 0;
            }

            int rotation = 1;
            //if (gamepad1.right_trigger) {

            //} else {
            rotation = 1;
            //}
            //robot.setInClawRotation(rotation);



            robot.setVerticalPower(vertical);

            //moves vertical slides
            if(gamepad1.dpad_up){
                slide = robot.SLIDE_HIGH_RUNG;
            }else if (gamepad1.dpad_down){
                vertical = 1;
                slide = robot.SLIDE_START;
            }

            robot.setVerticalPower(vertical);
            robot.SetSlidePosition(slide);

            if (gamepad1.x) {
                horizontalOffset = 1;
            }
            else if (gamepad1.b) {
                horizontalOffset = 0;
            }

            robot.setInClawPosition(inClawOffset);
            robot.setHorizontalPosition(horizontalOffset);
            robot.setIntakePosition(extendOffset);

            if (gamepad1.dpad_left) {
                extendOffset = 0;

            } else if (gamepad1.dpad_right) {
                extendOffset = 1;
                horizontalOffset = 0;
            }

            robot.setIntakePosition(horizontalOffset);
            robot.setIntakePosition(extendOffset);

            // Send telemetry messages to explain controls and show robot status
            telemetry.addData("Drive", "Left Stick");
            telemetry.addData("Turn", "Right Stick");
            telemetry.addData("Slide Up/Down", "Dpad_Up & Dpad_Down");
            telemetry.addData("HSlide Up/Down", "Dpad_Right & Dpad_Left");
            telemetry.addData("Vertical", "Y & A Buttons");
            telemetry.addData("Horizontal", "B & X Buttons");
            telemetry.addData("-", "-------");

            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Turn Power",  "%.2f", turn);
            telemetry.addData("Slide Power",  "%.2f", slide);
            telemetry.addData("HSlidePower", "%.2f", hslide);
            telemetry.addData("Vertical Power", "%.2f", vertical);
            telemetry.addData("Horizontal Position", "%.2f", horizontalOffset);
            telemetry.addData("Hand Position",  "Offset = %.2f", extendOffset);
            telemetry.update();

            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
