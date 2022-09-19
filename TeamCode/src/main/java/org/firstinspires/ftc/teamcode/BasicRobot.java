package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Components.Logger;

public class BasicRobot{
    public static Logger logger;
    public static LinearOpMode op = null;
    public BasicRobot(LinearOpMode opMode){
        op = opMode;
        logger = new Logger();
    }
}
