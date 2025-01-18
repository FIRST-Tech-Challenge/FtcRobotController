package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        ThreeWheelConstants.forwardTicksToInches = 0.0007501824844455658;
        ThreeWheelConstants.strafeTicksToInches = 0.00075048398213655;
        ThreeWheelConstants.turnTicksToInches = 0.000747494636252621;
        ThreeWheelConstants.leftY = 7.4375;
        ThreeWheelConstants.rightY = -6.875;
        ThreeWheelConstants.strafeX = .875;
        ThreeWheelConstants.leftEncoder_HardwareMapName = "frontLeft";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "backRight";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "frontRight";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}




