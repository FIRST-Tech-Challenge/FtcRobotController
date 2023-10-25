package teleop;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Bot;

public class DriveTest extends LinearOpMode{

    private GamepadEx gp1;
    Bot bot;
    private double driveSpeed=1;

    @Override
    public void runOpMode() throws InterruptedException {

        //need to add motorTest
        DcMotorEx motor_fr = hardwareMap.get(DcMotorEx.class, "fr");
        DcMotorEx motor_fl = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx motor_br = hardwareMap.get(DcMotorEx.class, "br");
        DcMotorEx motor_bl = hardwareMap.get(DcMotorEx.class, "bl");
        gp1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            drive();
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
