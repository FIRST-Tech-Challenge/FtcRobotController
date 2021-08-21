package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Accesories.WobbleGoal;
import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

        @Autonomous(name= "Shot", preselectTeleOp = "OneGPTeleop")
        public class Shot extends LinearOpMode {
            @Override
            public void runOpMode(){
                Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, true, false);

                waitForStart();
                robot.goToPosition(-24,24,0,1.0);
                robot.goToPosition(-60,0,0,1);
                robot.shootHighGoal(3);
                robot.setVelocity(0,1000);
                robot.startIntake();
                robot.startTransfer();
                robot.goToPosition(-24,0,0,0.7);
                robot.stopIntake();
                robot.stopTransfer();
                robot.goToPosition(-60,0,0,0.5);
                robot.shootHighGoal(1);
        robot.goToPosition(-72,0,0,0.5);
        stop();
    }
}

