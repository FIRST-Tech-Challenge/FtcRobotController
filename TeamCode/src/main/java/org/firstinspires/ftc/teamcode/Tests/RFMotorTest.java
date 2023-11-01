package org.firstinspires.ftc.teamcode.Tests;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static java.lang.Double.max;
import static java.lang.Double.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

import java.util.ArrayList;

/**
 * Harry
 * program to tune ALL RFMotor constants from dashboard, including the auto-tuned ones, abstract so people just have to extend this
 */
public abstract class RFMotorTest extends LinearOpMode {
    BasicRobot robot;
    RFMotor motor;
    String name;
    int max = 3000, min = 0;
    DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    double kS = 0, kV = 0, kA = 0, maxUpVelo, maxDownVelo, maxAccel, maxDecel, resistance, kP, kD;

    public void initialize(String p_name, int p_max, int p_min, DcMotorSimple.Direction p_direction){
        initialize(p_name,"placehold", p_max, p_min, p_direction, false);
    }

    public void initialize(String p_name, String p_name2, int p_max, int p_min, DcMotorSimple.Direction p_direction, boolean dualMotor) {
        name = p_name;
        max = p_max;
        min = p_min;
        direction = p_direction;
        robot = new BasicRobot(this, false);
        if(dualMotor){
            motor = new RFDualMotor(p_name, p_name2, true);
    } else {
      motor = new RFMotor(p_name, true);
        }
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(direction);
        motor.setConstants(p_max, p_min, resistance, kS, kV, kA, maxUpVelo, maxDownVelo, maxAccel, maxDecel, kP, kD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setConstants(double p_max, double p_min, double p_resistance, double p_kS, double p_kV, double p_kA, double p_maxUpVelo,
                             double p_maxDownVelo, double p_maxAccel, double p_maxDecel, double p_kP, double p_kD) {
        motor.setConstants(p_max, p_min, p_resistance, p_kS, p_kV, p_kA, p_maxUpVelo, p_maxDownVelo, p_maxAccel, p_maxDecel, p_kP, p_kD);
    }

    public void auto() {
            motor.setPosition((max + min)*0.9,0);
            packet.put("MOVING", "up");
            while(!motor.isDone()) {
                motor.setPosition((max + min) * 0.9, 0);
                packet.put("MOVING", "up");
                packet.put("position", motor.getCurrentPosition());
                packet.put("velocity", motor.getVelocity());
                packet.put("targPosition", motor.getTarCurPos());
                packet.put("targVel", motor.getTargVel());
                robot.update();
            }
        motor.setPosition((max + min)*0.1,0);
        packet.put("MOVING", "down");

        while(!motor.isDone()) {
                motor.setPosition((max + min) * 0.1, 0);
            packet.put("MOVING", "down");
            packet.put("position", motor.getCurrentPosition());
            packet.put("velocity", motor.getVelocity());
            packet.put("targPosition",motor.getTarCurPos());
            packet.put("targVel", motor.getTargVel());
            robot.update();
            }
    }
}
