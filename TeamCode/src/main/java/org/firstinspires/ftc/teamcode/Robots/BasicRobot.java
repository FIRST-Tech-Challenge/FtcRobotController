package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.Logger;
import org.firstinspires.ftc.teamcode.Components.Queuer;

public class BasicRobot{
    public static Logger logger;
    public static LinearOpMode op = null;
    protected Queuer queuer = null;
    public BasicRobot(LinearOpMode opMode){
        op = opMode;
        logger = new Logger();
        logger.createFile("/RobotLogs/GeneralRobot", "Runtime    Component               " +
                "Function         Action");
        logger.createFile("gamepad", "Value Name Time");

        queuer = new Queuer();

    }
    public void stop(){
        logger.closeLog();
        op.stop();
    }
}
