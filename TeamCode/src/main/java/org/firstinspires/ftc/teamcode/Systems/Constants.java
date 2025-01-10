package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    //Arm Reach Position
    public static int ARM_MAX_POSITION_OFFSET = 1145;
    public static int UP_ARM_MAX_POSITION_OFFSET = -11550;



    // PID variables
    public static float KP = 1.5F;  // Proportional gain
    public static float KI = 0.04F;  // Integral gain
    public static float KD = 0.03F;  // Derivative gain

    //servo positions
    public static int SERVO_CLOSED = 270;
    public static int SERVO_OPEN = 110;
}
