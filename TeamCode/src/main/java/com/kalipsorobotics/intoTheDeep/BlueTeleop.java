package com.kalipsorobotics.intoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BlueTeleop extends RedTeleop {

    public BlueTeleop() {
        this.isRed = false;
        this.takeInYellow = true;
    }



}
