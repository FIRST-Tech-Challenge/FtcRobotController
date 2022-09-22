package org.firstinspires.ftc.teamcode.ebotsenums;

public enum EncoderModel {
    CTR(4096),
    REV(8192),
    VIRTUAL(8192);

    private int clicksPerRevolution;

    EncoderModel(int clicksIn){
        this.clicksPerRevolution = clicksIn;
    }

    public int getClicksPerRevolution() {
        return clicksPerRevolution;
    }
}
