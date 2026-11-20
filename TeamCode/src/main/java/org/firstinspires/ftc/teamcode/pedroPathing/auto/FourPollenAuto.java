package org.firstinspires.ftc.teamcode.pedroPathing.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.RobotVars;
import org.firstinspires.ftc.teamcode.mechaninisms.RobotStateHandler;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.utils.commands.AllianceColor;
import org.firstinspires.ftc.teamcode.utils.commands.GamepadController;


@Autonomous(name = "Four Pollen Auto (PEDRO NOT WORKING DO NOT USE)", group = "Examples")
public class FourPollenAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    public int pathStateA = 0; // -1 is default, useless. 0 is initial
    public int shootingDelay = 1;


    // Poses used to calculate paths

    private Path startToScorePath, scoreToEndPath;



    RobotVars robotVars = new RobotVars();
    RobotStateHandler robotStateHandler = new RobotStateHandler(hardwareMap, robotVars);
    GamepadController gamepadController = new GamepadController(gamepad1, gamepad2);



    public void buildPaths() {


        startToScorePath = line(robotVars.startPose, robotVars.scorePose)
                .linear(robotVars.startPose, robotVars.scorePose);

        scoreToEndPath = line(robotVars.scorePose, robotVars.endPose)
                .linear(robotVars.scorePose, robotVars.endPose);

    }











    public void autonomousPathUpdateA() {
                    /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

        switch (pathStateA) {
            case -1:
                break;
            case 0:

                // Moving to the firing position
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.follow(startToScorePath);
                    setPathStateA(1);
                }
                break;

            case 1:

                robotVars.isStopperDown = true;
                robotVars.isIntakeActive = true;
                robotVars.isTransferActive = true;
                robotVars.isOuttakeActive = true;

                if ( Math.abs(RobotStateHandler.outtake.outtakeMotor.getVelocity() - robotVars.constants.outtakeMotorSpeed) < robotVars.constants.allowedOuttakeSpeedError) {
                    setPathStateA(2);
                }
                break;

            case 2:

                robotVars.isStopperDown = false;

                if (pathTimer.seconds() >= shootingDelay) {
                    setPathStateA(3);
                }


            case 3:
                robotVars.isStopperDown = true;
                robotVars.isIntakeActive = false;
                robotVars.isTransferActive = false;
                robotVars.isOuttakeActive = false;

                setPathStateA(4);

                break;


            case 4:

                // Moving to the end position
                if (!follower.isBusy()) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.follow(scoreToEndPath);
                    setPathStateA(-1);
                }
                break;

        }
    }








    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathStateA(int pState) {
        pathStateA = pState;
        pathTimer.reset();
    }






    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();

        autonomousPathUpdateA();

        //run smth intake. run smth outtake. run smth transfer. run smth stopper

        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
        telemetry.addData(" ", " ");

        telemetry.addData("pathStateA: ", pathStateA);
        telemetry.addData("Timer: ", pathTimer.seconds());
        telemetry.update();
    }







    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.reset();


        follower = Constants.create(hardwareMap);
        buildPaths();
        follower.setPose(robotVars.startPose);

        robotStateHandler.Init();

    }







    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {

        // Change poses
        robotStateHandler.RunMenu(gamepadController);

        // Effects of changing poses
        if (gamepadController.setTeamToBlue() || gamepadController.setTeamToRed() || gamepadController.setStartPoseToWall() || gamepadController.setStartPoseToCorner()) {
            buildPaths();
            follower.setPose(robotVars.startPose);
        }

        telemetry.addData("TeleOp: ", (robotVars.allianceColor == AllianceColor.Red ? "Red " : "Blue ") + (robotVars.startsInCorner ? "Corner" : "Wall"));

        telemetry.update();
    }




    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.reset();
        setPathStateA(0);
    }




    @Override
    public void stop() {
    }

}