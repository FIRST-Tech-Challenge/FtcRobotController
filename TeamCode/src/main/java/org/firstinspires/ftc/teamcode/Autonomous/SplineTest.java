package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Components.OdometryChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "SplineTest", preselectTeleOp = "OneGPTeleop")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);
        waitForStart();
        robot.tripleSplineToPosition(1, 5, 10, 12, 20, 16, 30, 12, 40, 5, 50, 0.5);
        stop();
    }
}

