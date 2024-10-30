package org.firstinspires.ftc.teamcode.Opmode.DriveTuning;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Arm;
import org.firstinspires.ftc.teamcode.Hardware.Claw;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Slides;
import org.firstinspires.ftc.teamcode.Hardware.Wrist;
import org.firstinspires.ftc.teamcode.Opmode.Auto.league1auto;

import java.util.List;

import dalvik.system.DelegateLastClassLoader;

@Autonomous
@Config


public class X extends LinearOpMode {
    enum State{
        BONK1,
        BONK2,
        BONK3,
        BONK4,
        BONK5,
        BONK6,
        BONK7,
        BONK8,
        BONK9
    }
    enum Actions{
        ACTION1,
        ACTION2,
        ACTION3,
        ACTION4,
        ACTION5,
        ACTION6,
        REST

    }
    State currentState = State.BONK1;

    Actions currentAction = Actions.ACTION1;
    public static double xTarget = 10, yTarget = 0, rTarget = 0;
    boolean timeToggle = true;
    boolean actionToggle = true;
    double TimeStamp = 0;
    double ActionStamp = 0;
    int currentCycle = 0;
    ElapsedTime timer = new ElapsedTime();
    double oldTime = 0;
    double timeStamp = 0;
    public void runOpMode() throws InterruptedException {
        //bulk reading
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        Drivetrain drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Slides slides = new Slides(hardwareMap, drive.getSlidesMotor());
        Arm arm = new Arm(hardwareMap, drive.getArmMotor());
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        while(opModeInInit()){
            claw.close();

        }
        waitForStart();
        double frequency = 0;
        double loopTime = 0;
        while(opModeIsActive()) {
            drive.autonomous();

            switch(currentAction){
                case REST:
                    break;
                case ACTION1:
                    arm.deposit();
                    slides.preScore();
                    currentAction=Actions.ACTION2;
                    break;
                case ACTION2:
//                    slides.score();
                    if(actionToggle){//to be filled in...
                        ActionStamp = timer.milliseconds();
                        actionToggle = false;
                    }
                    if(timer.milliseconds() > ActionStamp + 1500){
                        slides.score();

                        actionToggle = true;
                        currentAction = Actions.ACTION3;
                    }
                    break;
                case ACTION3:
//                    wrist.deposit();
                    if(actionToggle){//to be filled in...
                        ActionStamp = timer.milliseconds();
                        actionToggle = false;
                    }
                    if(timer.milliseconds() > ActionStamp + 1000){
                        wrist.deposit();
                        actionToggle = true;
                        currentAction = Actions.ACTION4;
                    }
                    break;
                case ACTION4:
//                    claw.open();
                    if(actionToggle){//to be filled in...
                        ActionStamp = timer.milliseconds();
                        actionToggle = false;
                    }
                    if(timer.milliseconds() > ActionStamp + 500){
                        claw.open();

                        actionToggle = true;
                        currentAction = Actions.ACTION5;
                    }
                    break;
                case ACTION5:
                    if (actionToggle){
                        ActionStamp=timer.milliseconds();
                        actionToggle=false;
                    }
                    if (timer.milliseconds()>ActionStamp+500){
                        wrist.intake();
                        slides.preScore();
                    }

                    if (timer.milliseconds()>ActionStamp+1000){
                        arm.preTake();
                        actionToggle=true;
                        currentAction=Actions.REST;
                    }
                    break;
                case ACTION6:
                    if (actionToggle){
                        ActionStamp=timer.milliseconds();
                        actionToggle=false;
                    }

                    slides.setTargetSlidesPosition(7);
                    if (timer.milliseconds()>ActionStamp+500){
                        arm.intake();
                    }
                    if (timer.milliseconds()>ActionStamp+1000){

                        claw.close();
                        actionToggle=true;
                        currentAction=Actions.REST;
                    }



                    break;


            }
            switch(currentState){
                case BONK1:

                    drive.setTarget(new Pose2d(-10, 11, 0));

                        if(timeToggle){//to be filled in...
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;

                        }
                        if(timer.milliseconds() > TimeStamp + 500){
                            timeToggle = true;
                            currentAction = Actions.ACTION1;
                            currentState = State.BONK2;

                        }

                    break;
                case BONK2:

                    drive.setTarget(new Pose2d(-10, 11, -45));

                        if(timeToggle){//to be filled in...
                            TimeStamp = timer.milliseconds();
                            timeToggle = false;
                            currentAction = Actions.ACTION1;
                        }
                        if(timer.milliseconds() > TimeStamp + 4000){

                            timeToggle = true;
                            if(currentCycle ==0){
                                currentState = State.BONK3;

                            }else if(currentCycle ==1){
                                currentState = State.BONK4;
                            }else if(currentCycle == 2){
                                currentState = State.BONK5;
                            }
                            currentCycle++;
                        }

                    break;
                case BONK3:
                    drive.setTarget(new Pose2d(-9, 22, 0));
                    if(timeToggle){//to be filled in...
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 1000){
                        currentAction=Actions.ACTION6;
                    }
                    if(timer.milliseconds() > TimeStamp + 2500){
                        timeToggle = true;
                        currentState = State.BONK2;
                    }
                    break;
                case BONK4:
                    drive.setTarget(new Pose2d(-20, 22, 0));
                    if(timeToggle){//to be filled in...
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 1000){
                        currentAction=Actions.ACTION6;
                    }
                    if(timer.milliseconds() > TimeStamp + 2500){
                        timeToggle = true;
                        currentState = State.BONK2;
                    }
                    break;
                case BONK5:
                    drive.setTarget(new Pose2d(-4, 52, 0));
                    if(timeToggle){//to be filled in...
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 2000){
                        timeToggle = true;
                        currentState = State.BONK6;
                    }
                    break;
                case BONK6:
                    drive.setTarget(new Pose2d(-23, 51, 0));
                    if(timeToggle){//to be filled in...
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 2000){
                        timeToggle = true;
                        currentState = State.BONK7;
                    }
                    break;
                case BONK7:
                    drive.setTarget(new Pose2d(-23, 12, 0));
                    if(timeToggle){//to be filled in...
                        TimeStamp = timer.milliseconds();
                        timeToggle = false;
                    }
                    if(timer.milliseconds() > TimeStamp + 2000){
                        timeToggle = true;
                        currentState = State.BONK8;
                    }
                    break;
                case BONK8:
                    //drive.setTarget(new Pose2d(6, 27, -90)); alternate drive waypoint1
                    //drive.setTarget(new Pose2d(76, 27, -90)); alternate drive waypoint2
                    //drive.setTarget(new Pose2d(76, 48, -90));alternate drive waypoint3
                    //drive.setTarget(new Pose2d(76, 48, -33));alternate drive waypoint4
                    drive.setTarget(new Pose2d(5, 49, 0));

                    break;

            }
            double newTime = getRuntime();
            loopTime = newTime - oldTime;
            frequency = 1 / loopTime;
            oldTime = newTime;

            drive.update();
            slides.update();
            arm.update();
            telemetry.addData("x", drive.getPose().getX());
            telemetry.addData("y", drive.getPose().getY());
            telemetry.addData("heading", drive.getPose().getHeading());
            telemetry.addData("Hub loop Time: ", frequency);
            telemetry.addData("powerX", drive.Powers().getX());
            telemetry.addData("powerY", drive.Powers().getY());
            telemetry.addData("powerHeading", drive.Powers().getHeading());

            telemetry.addData("xOut", drive.PIDOutputs().getX());
            telemetry.addData("yOut", drive.PIDOutputs().getY());
            telemetry.addData("headingOut", drive.PIDOutputs().getHeading());
            telemetry.addData("current state:",currentState.name());
            telemetry.addData("current action:",currentAction.name());


            telemetry.update();
        }
    }
}
