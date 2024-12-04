package com.kalipsorobotics.actions.intake;

import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CRservo {
    CRservo linkageServo1;
    CRservo linkageServo2;
    OpModeUtilities opModeUtilities;

    public CRservo(CRservo linkageServo1, CRservo linkageServo2) {
        this.linkageServo1 = linkageServo1;
        this.linkageServo2 = linkageServo2;
    }

    public void forward() {

    }
}
