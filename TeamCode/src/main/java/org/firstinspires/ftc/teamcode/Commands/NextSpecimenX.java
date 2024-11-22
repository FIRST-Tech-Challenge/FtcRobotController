package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;


// command template
public class NextSpecimenX  {

    protected static double nextX;
    public static double dispNextX;

    // constructor
    public NextSpecimenX() {

        //nextX = 0.35;

        // add subsystem requirements (if any) - for example:
        //addRequirements(RobotContainer.drivesystem);
    }

    // This method is called once when command is started
    public static void initialize() {
        nextX = 0.35;
        dispNextX = nextX;

    }


    // This method to return the nextX value
    public static double getX() {

        nextX = nextX - 0.10;
        dispNextX = nextX;
        return nextX;
    }




}