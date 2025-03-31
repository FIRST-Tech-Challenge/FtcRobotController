package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.HockeyStick;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
@TeleOp(name = "A HockeyStick test")
public class HockeyStickTest extends LinearOpMode {
    GamepadEvents controller1;
    HockeyStick hockeyStick;
    DriveTrain driveTrain;
    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);
        hockeyStick = new HockeyStick(hardwareMap, "hockeyStick");
        driveTrain = new DriveTrain(hardwareMap,"frontLeft", "backLeft", "frontRight",
                "backRight");

        waitForStart();
        while (opModeIsActive()) {
//            driveTrain.cubedDrive(-controller1.left_stick_y, controller1.left_stick_x, controller1.right_stick_x);
            //UP Pos:-750
            //Down Pos: -1000
            if (controller1.a.onPress())
            {
                telemetry.addLine("[A] pressed");
                hockeyStick.setDown();
            }
            if (controller1.b.onPress())
            {
                telemetry.addLine("[B] pressed");
                hockeyStick.setUP();
            }

            hockeyStick.adjustPos(-controller1.left_stick_y);

            controller1.update();

            telemetry.addLine("Press[A] to set down");
            telemetry.addLine("Press[B] to set up");
            telemetry.addLine("hold [Left Joy Stick Y] to adjust set Posiitons");
            telemetry.addData("Stick position: ", hockeyStick.getPosition());
            telemetry.update();
        }
    }
}
