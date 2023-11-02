package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

/**
 * Warren
 * program to auto-tune some RFMotor constants
 */
public abstract class AutoRFMotorTest extends LinearOpMode {
    BasicRobot robot;
    DcMotorEx motor;
    String name;
    int max=3000, min=0;
    DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    double kG=0, kS=0, kV=0, kA=0, maxUpVelo, maxDownVelo, maxAccel, maxDecel, resistance;

    ArrayList<Double[]> data, data2;


    public void initialize(String p_name, int p_max, int p_min, DcMotorSimple.Direction p_direction){
        name = p_name;
        max = p_max;
        min = p_min;
        direction = p_direction;
        robot = new BasicRobot(this, false);
        motor = hardwareMap.get(DcMotorEx.class, p_name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(direction);
        data = new ArrayList<>();
        data2 = new ArrayList<>();
    }
    public void auto(){
        kG=0.2;
        while(motor.getVelocity()<5&&motor.getCurrentPosition()<10){
            motor.setPower(kG);
            kG+=0.002;
            LOGGER.log("kG: "+kG+ " velo: " + motor.getVelocity());
            robot.update();
        }
//        while(motor.getCurrentPosition()<200){
//            motor.setPower(kG);
//            robot.update();
//        }
//        double lastV = motor.getVelocity();
//        while(motor.getVelocity()>lastV){
//            kG-=0.00001;
//            kS+=0.00001;
//            motor.setPower(kG);
//            lastV = motor.getVelocity();
//            LOGGER.log("kG: "+kG +" kS: "+ kS+ "  velo: " + motor.getVelocity());
//            robot.update();
//        }
//        while(motor.getCurrentPosition()>10){
//            motor.setPower(-0.2+kG);
//            robot.update();
//        }
        double addedPower = min(1-kG,0.8);
        while(motor.getCurrentPosition()<max-20){
            motor.setPower(kG+0.8);
            data.add(new Double[] {BasicRobot.time, motor.getVelocity(), (double)motor.getCurrentPosition(), addedPower});
            LOGGER.log("time: "+data.get(data.size()-1)[0]+" velo:"+ data.get(data.size()-1)[1]+" pos: "+
                    data.get(data.size()-1)[2] + " addedPower:" + data.get(data.size()-1)[3]);
            robot.update();
        }
        maxUpVelo = 0;
        maxAccel = 0;
        Double[] lastVel = data.get(0);
        data.remove(0);
        for(var i : data){
            if(maxUpVelo<i[1]){
                maxUpVelo=i[1];
            }
            if((i[1]-lastVel[1])/(i[0]-lastVel[1])>maxAccel){
                maxAccel = (i[1]-lastVel[1])/(i[0]-lastVel[0]);
            }
            lastVel = i;
        }
        kV = addedPower/maxUpVelo;
        kA = 0.8/maxAccel;
        addedPower = max(-1-kG,-0.8);
        while(motor.getCurrentPosition()>min+20){
            motor.setPower(kG-0.8);
            data2.add(new Double[] {BasicRobot.time, motor.getVelocity(), (double)motor.getCurrentPosition(), addedPower});
            LOGGER.log("time: "+data2.get(data2.size()-1)[0]+" velo:"+ data2.get(data2.size()-1)[1]+" pos: "+
                    data2.get(data2.size()-1)[2] + " addedPower:" + data2.get(data2.size()-1)[3]);
            robot.update();
        }
        maxDownVelo = 0;
        maxDecel = 0;
        lastVel = data2.get(0);
        data2.remove(0);
        for(var i : data2){
            if(maxDownVelo>i[1]){
                maxDownVelo=i[1];
            }
            if((i[1]-lastVel[1])/(i[0]-lastVel[1])<maxDecel){
                maxDecel = (i[1]-lastVel[1])/(i[0]-lastVel[0]);
            }
            lastVel = i;
        }
        resistance = kG/kV;
        packet.put("kG", kG);
        packet.put("kS", kS);
        packet.put("kV", kV);
        packet.put("kA", kA);
        packet.put("maxUpVelo", maxUpVelo);
        packet.put("maxDownVel", maxDownVelo);
        packet.put("maxAccel", maxAccel);
        packet.put("maxDecel", maxDecel);
        packet.put("resistance", resistance);
        
        LOGGER.log("kG:" + kG);
        LOGGER.log("kS:" + kS);
        LOGGER.log("kV: "+ kV);
        LOGGER.log("kA: "+ kA);
        LOGGER.log("maxUpVelo: "+ maxUpVelo);
        LOGGER.log("maxDownVelo: "+ maxDownVelo);
        LOGGER.log("maxAccel: "+ maxAccel);
        LOGGER.log("maxDecel: "+ maxDecel);
        LOGGER.log("resistance : "+ resistance);
        robot.update();
        stop();
    }
}
