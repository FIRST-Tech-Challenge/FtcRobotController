package org.firstinspires.ftc.teamcode.pedroPathing.auto; // make sure this aligns with class location

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
public class ExampleAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private Path scorePreload;
    private Path pathA, pathB;










    private final Pose startPose = new Pose(28.5, 128,   Math.toRadians(180)  );
    private final Pose scorePose = new Pose(60, 85,   Math.toRadians(135)  );


    public void buildPaths() {

        pathA = newPath(startPose, scorePose);
        pathB = newPath(scorePose, startPose);

    }



    public void autonomousPathUpdate() {
        switch (pathState) {

            // First part of the path
            case 0:

                runToPath(pathA);
                break;

            // Second part of the path
            case 1:

                runToPath(pathB);
                break;

            // End of the path
            default:

                break;
        }
    }











    public void runToPath(Path inputPath) {
        if (!follower.isBusy()) {
            follower.follow(inputPath);

            pathState++;
            pathTimer.reset();
        }
    }

    public Path newPath(Pose start, Pose end) {
        return line(start, end).linear(start, end);
    }


    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.reset();
    }






    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("heading", follower.pose().heading());
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
        follower.setPose(startPose);

    }







    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }




    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.reset();
        setPathState(0);
    }




    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }

}