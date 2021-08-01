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
        robot.tripleSplineToPosition(1, 0, 0, 12, 20, 16, 30, 12, 40, 5, 50, 1.0);
        //robot.tripleSplineToPosition(1, 0, 0, 0, 20, 0, 30, 0, 40, 0, 50, 1.0);
        //robot.tripleSplineToPosition(1, 0, 0, 20, 12, 30, 16, 40, 12, 50, 5, 0.8);
        //robot.tripleSplineToPosition(1, 0, 0, 20, 0, 30, 0, 40, 0, 50, 0, 0.7);
        stop();
    }
}

