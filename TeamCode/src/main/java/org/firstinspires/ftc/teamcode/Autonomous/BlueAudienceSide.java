package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;

public class BlueAudienceSide extends LinearOpMode {

    RobotClass robot = new RobotClass(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        //Moving to spike mark grid square
        robot.move(0.5, 50);

        //turning and moving to backdrop
        robot.gyroTurning(90);

        //parking
    }
}
