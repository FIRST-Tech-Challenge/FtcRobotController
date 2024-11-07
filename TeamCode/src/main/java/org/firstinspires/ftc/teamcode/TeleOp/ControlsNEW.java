package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ControlsNEW extends LinearOpMode {

    public final RobotHardware robot = new RobotHardware();
    public Base driveControl;

    public Claw clawControl;
    public Hang hangControl;

    
    @Override

    public void runOpMode() {
        robot.init(hardwareMap); // Initialize robot hardware
        driveControl = new Base(robot); // Initialize Control1 with RobotHardware
        clawControl = new Claw(robot); // Initialize Control2 with RobotHardware
        hangControl = new Hang(robot); // Initialize Hang with RobotHardware

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Pass the joystick values to Control1's drive method
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x * 1.1; // Strafing
            double rx = gamepad1.right_stick_x; // Rotation
            driveControl.drive(y, x, rx);

            clawControl.controlClaw(
                    gamepad2.a,
                    gamepad2.y,
                    gamepad2.x,
                    gamepad2.b,
                    gamepad2.right_stick_button,
                    gamepad2.left_bumper,
                    gamepad2.right_bumper
            );

            hangControl.controlHang(
                    gamepad2.dpad_down,
                    gamepad2.dpad_up
            );
        }
    }
}



// V1 (04/16/2024) Preparing for Next Year, I improved the base programming to rely on math instead
// of power for it to be more reliable when shifting, having my team mates have more control over
// the robot, and lastly, make the robot more smooth when driving it.
// V2 (09/07/2024) New school year with new team and finalize the "plan" for "Into the Deep" FTC Game
// We initialized with Auto, trying to go to the baskets and making the blocks into the High Basket.
// We might be able to make the pre-loaded block into the high basket as that will be the easiest thing to do.
// However,