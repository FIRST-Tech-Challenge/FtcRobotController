package org.firstinspires.ftc.teamcode.drive.Autonomous.AutonomousScripts;

import static org.firstinspires.ftc.teamcode.drive.modules.riptideUtil.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.drive.modules.riptideUtil.CLAW_OPEN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.drive.Autonomous.Path;
import org.firstinspires.ftc.teamcode.drive.Autonomous.Waypoint;
import org.firstinspires.ftc.teamcode.drive.modules.Timer;
import org.firstinspires.ftc.teamcode.drive.modules.riptideUtil;

// this could be usefull for testing auton
// ----- READY TO TRANSFER ----- //

@Autonomous
public class Test extends LinearOpMode {

    AutonomousRobot robot;

    Path p = new Path.PathBuilder()
            .addNewFullPoint(
                    new Waypoint(24, 48, -90, 96, 72, DistanceUnit.INCH),
                    Path.FollowMethods.FOLLOW_AND_TURN,
                    () -> {
                        robot.setSlideGoalPosition(2500);
                        robot.setSlidePower();
                        robot.clawArm.setAllThreeJoints(70, 90, 0);
                    },
                    1
            )
            .addNewFullPoint(
                    new Waypoint(24, 48, -90, 96, 72, DistanceUnit.INCH),
                    Path.FollowMethods.FOLLOW_AND_TURN,
                    () -> {


                        robot.setSlidePower();
                        Timer t = new Timer();
                        robot.setClawPos(CLAW_OPEN);

                        if(t.amountSecondsPassed(1) && !t.amountSecondsPassed(1.5)){
                            robot.setClawPos(CLAW_CLOSE);
                        }
                        else if(t.amountSecondsPassed(2)){
                            robot.setClawPos(CLAW_OPEN);
                        }
                    },
                    3
            )
            .addNewFullPoint(
                    new Waypoint(0,0,90,72, 96, DistanceUnit.INCH),
                    Path.FollowMethods.FOLLOW_AND_TURN,
                    () -> {
                        robot.setSlidePower();
                        robot.clawArm.setAllThreeJoints(90, 0, 0);
                    },
                    0
            )
            .build();


    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION            *
         * * * * * * * * * * * * * * *
         */

        robot = new AutonomousRobot(hardwareMap);

        while(Math.abs(riptideUtil.SLIDE_INIT_POS - robot.getOneSlidePosition()) > 10 ){
            robot.setSlideGoalPosition(riptideUtil.SLIDE_INIT_POS);
            robot.setSlidePower();
        }

        robot.setSlidePower(0);

        robot.clawArm.setAllThreeJoints(riptideUtil.SLIDE_PIVOT_INIT_ANGLE, riptideUtil.ARM_INIT_ANGLE, riptideUtil.WRIST_INIT_ANGLE);

        telemetry.addData("Robot status", "successfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // * * * * * * * * * * * * * * *
        // * Start button clicked
        // * * * * * * * * * * * * * * *

        telemetry.clear();
        robot.startOdometry();
        robot.setPath(p);
        robot.startPath();
        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */

        while(opModeIsActive()){
            robot.step();
            telemetry.addData("at point", robot.atPoint(robot.getCurrPos(),new Waypoint(24, 48, -90, 96, 72, DistanceUnit.INCH)));
            telemetry.addData("ElapsedTime", robot.getElapsedTime());
            telemetry.addData("Delay Time", robot.getDelay());
            telemetry.addData("path index", robot.getPathIndex());
            telemetry.addData("Time", robot.getTime());
            telemetry.addData("at point", robot.isAtPoint());
            telemetry.addData("Size", robot.pathsize());
            telemetry.addData("current Waypoint", robot.autonPos());
            telemetry.addData("CURRENTPOINT", robot.autonrealPos());

            telemetry.addLine("\n");
            telemetry.addData("past Delay until next point", robot.pastDelayUntilNextPoint());
            telemetry.addData("not at end of path?", robot.isNotAtEndOfPath());
            telemetry.update();
        }


    }

}
