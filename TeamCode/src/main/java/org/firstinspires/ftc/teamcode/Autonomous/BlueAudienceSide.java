package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous(name = "BlueAudienceSide", group = "Autonomous")
public class BlueAudienceSide extends LinearOpMode {

    RobotClass robot = new RobotClass(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        //Moving to spike mark grid square
        robot.moveNoEncoders(0.5, 0.5, 1250);

        //turning and moving to backdrop
        robot.gyroTurning(90);

        //TODO: Fix turning code (currently not turning)


        //parking
    }
}
