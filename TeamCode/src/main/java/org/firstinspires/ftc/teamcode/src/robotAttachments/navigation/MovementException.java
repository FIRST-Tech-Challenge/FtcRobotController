package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation;

import com.qualcomm.robotcore.util.RobotLog;

public class MovementException extends Exception {
    public MovementException(String message) {
        super(message);
        RobotLog.addGlobalWarningMessage(message);
    }
}
