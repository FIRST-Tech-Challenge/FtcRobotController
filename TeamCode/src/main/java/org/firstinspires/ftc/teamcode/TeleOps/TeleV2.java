package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareMap.HMap;
import org.firstinspires.ftc.teamcode.TeleOps.TeleBase.TeleStates;

@TeleOp(name="STATE_MACHINE_TELEOP", group = "TELE_PROTO")
public class TeleV2 extends LinearOpMode {
    public HMap robot = new HMap();
    TeleStates ts;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        ts = new TeleStates(robot, gamepad1);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            ts.change_state(ts.IDLE);
            ts.change_state(ts.MONITOR_A);
            ts.change_state(ts.MONITOR_Y);
            ts.change_state(ts.MONITOR_LS);
            ts.change_state(ts.MONITOR_RS);
        }


    }
}
