package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.00197;
        ThreeWheelConstants.strafeTicksToInches = 0.00196;
        ThreeWheelConstants.turnTicksToInches = 0.0021;
        ThreeWheelConstants.leftY = 4.5;
        ThreeWheelConstants.rightY = -4.5;
        ThreeWheelConstants.strafeX = -6.75;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftRear";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
    }
}




