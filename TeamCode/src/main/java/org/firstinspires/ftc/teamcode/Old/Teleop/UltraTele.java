package org.firstinspires.ftc.teamcode.Old.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static java.lang.Math.abs;
import static java.lang.Math.floor;
import static java.lang.Math.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Logger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.Robots.PwPRobot;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;


@TeleOp(name= "UltraTest")
public class UltraTele extends LinearOpMode {
    private AnalogInput ultrasonicFront, ultrasonicBack, ultrasonicRight, ultrasonicLeft;
    private LED ultraFront, ultraBack, ultraRight, ultraLeft;
    private ArrayList<Double> tempdata = new ArrayList<>();
    private ArrayList<Double> outputMean = new ArrayList<>();
    private ArrayList<Double> outputDeviation = new ArrayList<>();
    boolean leftbool = false; boolean rightbool = false;
    boolean press = false;
    int timesPressed = 0;
    BasicRobot robot;
    @Override
    public void runOpMode(){
        robot = new BasicRobot(this,true);
        SampleMecanumDrive roadrun = new SampleMecanumDrive(this.hardwareMap);
        roadrun.setPoseEstimate(new Pose2d(-36,12, Math.toRadians(270)));
        ElapsedTime op = new ElapsedTime();
        logger.createFile("/RobotLogs/sanicM", "0 degrees,5 degrees,10 degrees,15 degrees,20 degrees,25 degrees,30 degrees");
        logger.createFile("/RobotLogs/sanicD", "0 degrees,5 degrees,10 degrees,15 degrees,20 degrees,25 degrees,30 degrees");

        ultrasonicLeft = hardwareMap.get(AnalogInput.class, "ultrasonicLeft");
        ultrasonicRight = hardwareMap.get(AnalogInput.class, "ultrasonicRight");
        double angle;
        double globalx = 0; double globaly = 0;
        double globalAngle = 0;

//        ultraRight = hardwareMap.get(LED.class, "ultraRight");
        ultraLeft = hardwareMap.get(LED.class, "ultraLeft");

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        waitForStart();
        // arraylist for data, each time we un-press dpad calculate mean + deviation & add to output arraylists & clear data
        // 2 arraylists for output, 1 for mean and 1 for standard deviation
        // 4 distances, 7 angles, use button A for distances, use button Y for angles
        // keep adding int to angles, reset when distance gets changed
        // store a string, log that + newline going to new distance
        while(opModeIsActive()){
            if(floor(getRuntime())%2==0){
//                ultraRight.enable(false);
                ultraLeft.enable(false);
            }
            else{
//                ultraRight.enable(true);
                ultraLeft.enable(true);
            }
            if(gamepad1.dpad_right){
                tempdata.add(90.48337 * ultrasonicRight.getVoltage());
                rightbool = true;
            }
            if(!gamepad1.dpad_right && rightbool == true){
                outputMean.add(getMean(tempdata));
                outputDeviation.add(getDev(tempdata));
                tempdata.clear();
                rightbool = false;
            }
            if(gamepad1.dpad_left){
                tempdata.add(90.48337 * ultrasonicRight.getVoltage());
                leftbool = true;
            }
            if(!gamepad1.dpad_left && leftbool == true){
                outputMean.add(getMean(tempdata));
                outputDeviation.add(getDev(tempdata));
                tempdata.clear();
                leftbool = false;
            }
            if(gamepad1.right_bumper && press == false){
                press = true;
                for(int i=0;i<outputMean.size();i++){
                    logger.logNoTime("/RobotLogs/sanicM", outputMean.get(i) + "," + outputDeviation.get(i));
                }
            }
            angle = Math.asin((Math.abs((90.48337 * ultrasonicRight.getVoltage() - 13.12465) - (90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))/5);
            double goofy = (((90.48337 * ultrasonicRight.getVoltage() - 13.12465) + (90.48337 * ultrasonicLeft.getVoltage() - 13.12465))/2);
            if(!gamepad1.right_bumper && press == true){
                press = false;
            }
            if(roadrun.getPoseEstimate().getHeading() < Math.toRadians(315) && roadrun.getPoseEstimate().getHeading() > Math.toRadians(225)){ // up, -Y
                globaly = (0+goofy) + (7*sin(angle));
                globalx = 0;
                if(((90.48337 * ultrasonicRight.getVoltage() - 13.12465) < ((90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))){
                    globalAngle = Math.toRadians(270) + angle;
                }
                if(((90.48337 * ultrasonicRight.getVoltage() - 13.12465) > ((90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))){
                    globalAngle = Math.toRadians(270) - angle;
                }
            }
            if(roadrun.getPoseEstimate().getHeading() < Math.toRadians(225) && roadrun.getPoseEstimate().getHeading() > Math.toRadians(135)){ //right, -X
                globalx = (-70.5+goofy) + (7*sin(angle));
                globaly = 0;
                if(((90.48337 * ultrasonicRight.getVoltage() - 13.12465) < ((90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))){
                    globalAngle = Math.toRadians(180) + angle;
                }
                if(((90.48337 * ultrasonicRight.getVoltage() - 13.12465) > ((90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))){
                    globalAngle = Math.toRadians(180) - angle;
                }
            }
            if(roadrun.getPoseEstimate().getHeading() < Math.toRadians(135) && roadrun.getPoseEstimate().getHeading() > Math.toRadians(45)){ //down, +Y
                globaly = (70.5-goofy) - (7*sin(angle));
                globalx = 0;
                if(((90.48337 * ultrasonicRight.getVoltage() - 13.12465) < ((90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))){
                    globalAngle = Math.toRadians(90) + angle;
                }
                if(((90.48337 * ultrasonicRight.getVoltage() - 13.12465) > ((90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))){
                    globalAngle = Math.toRadians(90) - angle;
                }
            }
            if(roadrun.getPoseEstimate().getHeading() < Math.toRadians(45) || roadrun.getPoseEstimate().getHeading() < Math.toRadians(360) && roadrun.getPoseEstimate().getHeading() > Math.toRadians(315)){ //left, +X
                globalx = (70.5-goofy) - (7*sin(angle));
                globaly = 0;
                if(((90.48337 * ultrasonicRight.getVoltage() - 13.12465) < ((90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))){
                    globalAngle = Math.toRadians(0) + angle;
                }
                if(((90.48337 * ultrasonicRight.getVoltage() - 13.12465) > ((90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))){
                    globalAngle = Math.toRadians(0) - angle;
                }
            }
            telemetry.addData("right", 90.48337 * ultrasonicRight.getVoltage() - 13.12465);
            telemetry.addData("left", 90.48337 * ultrasonicLeft.getVoltage() - 13.12465);
            telemetry.addData("goofy", ((90.48337 * ultrasonicRight.getVoltage() - 13.12465) + (90.48337 * ultrasonicLeft.getVoltage() - 13.12465))/2);
            telemetry.addData("angle", Math.asin((Math.abs((90.48337 * ultrasonicRight.getVoltage() - 13.12465) - (90.48337 * ultrasonicLeft.getVoltage() - 13.12465)))/5));
            telemetry.addData("global", "(" + globalx + "," + globaly + ")");
//            telemetry.addData("right", ultraRight.isLightOn());
            telemetry.addData("global angle", globalAngle);
            telemetry.addData("left", ultraLeft.isLightOn());
            telemetry.update();
        }
        stop();
    }
    public double getMean(ArrayList<Double> data){
        double n = 0;
        for(int i=0;i<data.size();i++){
            n+= data.get(i);
        }
        return n/data.size();
    }
    public double getDev(ArrayList<Double> data){
        double n = 0;
        for(int i=0;i<data.size();i++){
            n+= Math.pow((Math.abs(data.get(i) - getMean(data))), 2);
        }
        return Math.sqrt(n/data.size());
    }



}
