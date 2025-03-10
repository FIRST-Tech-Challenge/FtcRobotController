package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0005390465353908967;
        ThreeWheelConstants.strafeTicksToInches = 0.0008312127344516645;
        ThreeWheelConstants.turnTicksToInches = 0.0006781844656416374;
        ThreeWheelConstants.leftY = 3.5;
        ThreeWheelConstants.rightY = -3.5;
        ThreeWheelConstants.strafeX = 0.1;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "odol";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "odor";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "odom";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




