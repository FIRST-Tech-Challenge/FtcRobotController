package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes;

import org.firstinspires.ftc.teamcode.ebotsenums.Accuracy;
import org.firstinspires.ftc.teamcode.ebotsenums.EncoderSetup;
import org.firstinspires.ftc.teamcode.ebotsenums.GyroSetting;
import org.firstinspires.ftc.teamcode.ebotsenums.SoftStart;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;

public enum AutonParameters {
    SIMULATED_TWO_WHEEL(Speed.FAST, GyroSetting.NONE, Accuracy.STANDARD, SoftStart.MEDIUM, EncoderSetup.TWO_WHEELS),
    SIMULATED_THREE_WHEEL(Speed.FAST, GyroSetting.NONE, Accuracy.STANDARD, SoftStart.MEDIUM, EncoderSetup.THREE_WHEELS),
    DEBUG_TWO_WHEEL(Speed.SLOW, GyroSetting.EVERY_LOOP, Accuracy.STANDARD, SoftStart.SLOW_START, EncoderSetup.TWO_WHEELS),
    DEBUG_THREE_WHEEL(Speed.SLOW, GyroSetting.NONE, Accuracy.STANDARD, SoftStart.SLOW_START, EncoderSetup.THREE_WHEELS),
    CALIBRATION_TWO_WHEEL(Speed.FAST, GyroSetting.EVERY_LOOP, Accuracy.STANDARD, SoftStart.SLOW_START, EncoderSetup.THREE_WHEELS),
    STANDARD_TWO_WHEEL(Speed.MEDIUM, GyroSetting.NONE, Accuracy.STANDARD, SoftStart.MEDIUM, EncoderSetup.TWO_WHEELS),
    STANDARD_THREE_WHEEL(Speed.MEDIUM, GyroSetting.EVERY_LOOP, Accuracy.STANDARD, SoftStart.MEDIUM, EncoderSetup.THREE_WHEELS),
    COMPETITION(Speed.FAST, GyroSetting.EVERY_LOOP, Accuracy.STANDARD, SoftStart.SLOW_START, EncoderSetup.COMPETITION_BOT);

    private Speed speed;
    private GyroSetting gyroSetting;
    private Accuracy accuracy;
    private SoftStart softStart;
    private EncoderSetup encoderSetup;

    AutonParameters(Speed speedIn, GyroSetting gyroIn, Accuracy accuracyIn, SoftStart softStartIn, EncoderSetup encoderSetupIn){
        this.speed = speedIn;
        this.gyroSetting = gyroIn;
        this.accuracy = accuracyIn;
        this.softStart = softStartIn;
        this.encoderSetup = encoderSetupIn;
    }

    public Speed getSpeed() {
        return speed;
    }

    public GyroSetting getGyroSetting() {
        return gyroSetting;
    }

    public Accuracy getAccuracy() {
        return accuracy;
    }

    public SoftStart getSoftStart() {
        return softStart;
    }

    public EncoderSetup getEncoderSetup() {
        return encoderSetup;
    }

    public void setSpeed(Speed speedIn){
        this.speed = speedIn;
    }

    public void setEncoderSetup(EncoderSetup encoderSetupIn) {
        this.encoderSetup = encoderSetupIn;
    }

    public boolean usesSimulatedEncoders(){
        boolean returnValue = false;
        if(this == AutonParameters.SIMULATED_TWO_WHEEL || this==AutonParameters.SIMULATED_THREE_WHEEL){
            returnValue = true;
        }
        return returnValue;
    }

    @Override
    public String toString(){
        String separator = " | ";
        StringBuilder sb = new StringBuilder();
        sb.append(speed.toString());
        sb.append(separator);
        sb.append(gyroSetting.toString());
        sb.append(separator);
        sb.append(accuracy.toString());
        sb.append(separator);
        sb.append(softStart.toString());
        sb.append(separator);
        sb.append(encoderSetup.toString());

        return sb.toString();
    }
}
