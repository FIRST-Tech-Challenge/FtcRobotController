package org.firstinspires.ftc.teamcode.samples;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/** Reference follower-driven teleop. Disabled until Constants can build a follower. */
@Disabled
@TeleOp
public class PedroTeleOpSample extends CommandOpMode {

    public Follower follower;
    public TelemetryData telemetryData = new TelemetryData(telemetry);

    @Override
    public void initialize() {
        follower = Constants.createFollower(hardwareMap);
        super.reset();

        // Pedro 3 has no startTeleopDrive(): the follower enters manual mode on the first
        // manual() call and leaves it when a path is followed.
    }

    @Override
    public void run() {
        super.run();

        /* Robot-Centric Drive
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        */

        // Field-Centric Drive
        follower.manual(
                ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        follower.pose().heading()
                )
        );
        follower.update();

        telemetryData.addData("X", follower.pose().x());
        telemetryData.addData("Y", follower.pose().y());
        telemetryData.addData("Heading", follower.pose().heading());
        telemetryData.update();
    }
}
