package org.firstinspires.ftc.teamcode.Systems;

public interface Constants {
    //Arm Reach Position
    int ARM_MAX_POSITION_OFFSET = 1145;



    // PID variables
    float KP = 1.5F;  // Proportional gain
    float KI = 0.04F;  // Integral gain
    float KD = 0.03F;  // Derivative gain

    //servo positions
    int SERVO_CLOSED = 270;
    int SERVO_OPEN = 110;
}
