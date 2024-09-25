package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp (name = "Robot Centric Drive", group = "Robot")

public class RobotCentric extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware. Prefix any hardware function with "robot." to
    // access this class.
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double axial = 0;
        double lateral = 0;
        double yaw = 0;
//        double arm = 0;
//        double handOffset = 0;

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
            lateral = gamepad1.left_stick_x;
            yaw = gamepad1.right_stick_x;

            // While right bumper is held slow mode is activated
            if (gamepad1.right_bumper) {
                axial = -gamepad1.left_stick_y * 0.25; // Note: pushing stick forward gives negative value
                lateral = gamepad1.left_stick_x * 0.25;
                yaw = gamepad1.right_stick_x * 0.25;
            }

            // Combine drive and turn for blended motion. Use RobotHardware class
            robot.driveRobot(axial, lateral, yaw);

            //Pace this loop so hands move at a reasonable speed.
            sleep( 50);

        }
    }
}
