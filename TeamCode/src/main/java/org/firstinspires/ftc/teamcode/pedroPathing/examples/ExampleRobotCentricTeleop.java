package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import android.provider.SyncStateContract;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and robot-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "Example Robot-Centric Teleop", group = "Examples")
public class ExampleRobotCentricTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);

    private DistanceSensor rearDistanceSens;
    double distance;
    private ElapsedTime relocalizeTimer;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class,LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        rearDistanceSens = hardwareMap.get(DistanceSensor.class, "rearDistanceSens");
        relocalizeTimer = new ElapsedTime();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        distance = rearDistanceSens.getDistance(DistanceUnit.INCH);
//        if(((int)relocalizeTimer.milliseconds()) % 100 == 0) {
//            if(Math.abs(follower.getPose().getX() - distance) < 4) {
//                relocalizeWithDistanceSensor();
//            }
//        }

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: true
        */

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("Rear Distance (INCHES)", distance);
        telemetry.addLine();
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

    }

    private void relocalizeWithDistanceSensor(){
        if(distance < 0.5){
            follower.setPose(new Pose(distance, follower.getPose().getY(), follower.getPose().getHeading()));
        }
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}