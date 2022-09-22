package org.firstinspires.ftc.teamcode.ebotsenums;

public enum EncoderSetup {
    //TODO:  Refactor to wire into RobotDesign
    /**
     * This enumeration captures the setup of the encoder
     * This is typically determined by
     *      Desired Control Strategy
     *      RobotDesign (encoder model)
     */
    TWO_WHEELS(RobotOrientation.NONE, EncoderModel.CTR)
    , THREE_WHEELS(RobotOrientation.FORWARD, EncoderModel.CTR)
    , COMPETITION_BOT(RobotOrientation.NONE, EncoderModel.REV);

    private RobotOrientation doubleEncoderDirection;
    private EncoderModel encoderModel;

    EncoderSetup(RobotOrientation robotOrientation, EncoderModel encoderModelIn){
        //Note: returns null if two wheel
        this.doubleEncoderDirection = robotOrientation;
        this.encoderModel = encoderModelIn;
    }

    public RobotOrientation getDoubleEncoderDirection() {return doubleEncoderDirection;}

    public EncoderModel getEncoderModel() { return encoderModel; }

    // Forward encoder:  backLeft
    // Lateral encoder:  frontLeft
}
