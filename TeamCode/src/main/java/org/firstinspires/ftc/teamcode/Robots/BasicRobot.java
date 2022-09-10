package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.Logger;
import org.firstinspires.ftc.teamcode.Components.Queuer;

public class BasicRobot{
    public static Logger logger;
    public static LinearOpMode op = null;
    private Queuer queuer = null;
    public BasicRobot(LinearOpMode opMode){
        op = opMode;
        logger = new Logger();
        logger.createFile("gamepad", "Value Name Time");
        queuer = new Queuer();
    }
    public void stop(){
        logger.closeLog();
        op.stop();
    }
    public void readGamepad(float value, String name){
        logger.log("gamepad", name + ": " + value + "at time " + op.time);
    }
    public void readGamepad(int value, String name){
        logger.log("gamepad", name + ": " + value + "at time " + op.time);
    }
    public void readGamepad(double value, String name){
        logger.log("gamepad", name + ": " + value + "at time " + op.time);
    }
    public void readGamepad(boolean value, String name){
        logger.log("gamepad", name + ": " + value + "at time " + op.time);
    }
}
