package org.firstinspires.ftc.teamcode.Opmode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Usefuls.Math.M;

import java.util.Locale;

@Autonomous
public class league1auto extends LinearOpMode{

    enum State{
        DEPOSIT,
        BLOCK1,
        BLOCK2,
        PARK,
        FINISH
    }
    Pose2d startPosition = new Pose2d(-37, -60, Math.toRadians(90));
    Pose2d deposit = new Pose2d(-50, -52, Math.toRadians(45));
    Pose2d block1 = new Pose2d(-50, -32, Math.toRadians(90));
    Pose2d block2 = new Pose2d(-55, -32, Math.toRadians(90));
    Pose2d park = new Pose2d(-35, -15, Math.toRadians(90));

    int cycle = 2; //how many blocks ur picking upp
    int currentCycle =0;
    State currentState = State.DEPOSIT;
    //timing logic for future actions
    boolean timeToggle = true;
    double TimeStamp = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain drive = new Drivetrain(hardwareMap, startPosition);
        //doesnt work lol
//        drive.setStartPostion(startPosition);

        waitForStart();
        while(opModeIsActive()){
            switch(currentState){
                case DEPOSIT:
                    drive.setZeroMoveAngle(Math.toRadians(30));
                    drive.setTarget(deposit);
                    if(drive.isAtTarget()){
                        if(timeToggle){//to be filled in...
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + 2000){
                            timeToggle = true;
                            currentCycle++;
                            if(currentCycle == 1)
                                currentState = State.BLOCK1;
                            if(currentCycle == 2)
                                currentState = State.BLOCK2;
                        }
                    }
                    break;
                case BLOCK1:
                    drive.setZeroMoveAngle(Math.toRadians(10));
                    drive.setTarget(block1);
                    if(drive.isAtTarget()){
                        if(timeToggle){//to be filled in...
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if(timer.milliseconds() > TimeStamp + 2000){
                            timeToggle = true;
                            currentState = State.DEPOSIT;
                        }
                    }
                    break;
                case BLOCK2:
                    drive.setZeroMoveAngle(Math.toRadians(10));
                    drive.setTarget(block2);
                    if(drive.isAtTarget()) {
                        if (timeToggle) {//to be filled in...
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if (timer.milliseconds() > TimeStamp + 2000) {
                            timeToggle = true;
                            currentState = State.PARK;
                        }
                    }
                    break;
                case PARK:
                    drive.setTarget(park);
                    if(drive.isAtTarget()) {
                        if (timeToggle) {//to be filled in...
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                        }
                        if (timer.milliseconds() > TimeStamp + 2000) {
                            timeToggle = true;
                            currentState = State.FINISH;
                        }
                    }
                    break;
                case FINISH:
                    break;
            }
            drive.update();
            telemetry.addData("STATE", currentState);
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", drive.getPose().getX(), drive.getPose().getY(), M.toDegrees(drive.getPose().getHeading()));
            telemetry.addData("POSITION", data);
            telemetry.update();
        }
    }
}
