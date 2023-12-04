package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous()

public class BlueAudienceSide extends LinearOpMode {

    RobotClass robot = new RobotClass(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        //Moving to spike mark grid square

        //Move forwards one more square

        //turning 90 degrees

        //move forwards to the wall

    }
}
