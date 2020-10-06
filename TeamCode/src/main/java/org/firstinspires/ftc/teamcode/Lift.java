package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    //This class wil contain the lift motor
    //Methods will include an up/down move, and maybe others
    /******************************************************
     * Decisions that need to be made
     *
     * How to limit upward motion
     * Assuming we start from the bottom every time. The only way to get down is to go all the way.
     * How to return enum from getLiftPosition
     *****************************************************/
/*
    public DcMotor frontLift = null;
    public DcMotor backLift  = null;

    HardwareMap hwMap        = null;

    DigitalChannel REVTouchBottom;

    static final double SPEED     = -.75;
    static final int LEVEL_INSIDE = -475;
    static final int ADJUSTED     = -200;
    static final int PLATE_HEIGHT = -450;
    static final int BLOCK_HEIGHT = -1000;
    static final int LEVEL_TWO    = PLATE_HEIGHT+ADJUSTED;
    static final int LEVEL_THREE  = BLOCK_HEIGHT+PLATE_HEIGHT+ADJUSTED;
    static final int LEVEL_FOUR   = (BLOCK_HEIGHT*2)+PLATE_HEIGHT+ADJUSTED;
    static final int LEVEL_FIVE   = (BLOCK_HEIGHT*3)+PLATE_HEIGHT+ADJUSTED;
    static final int LEVEL_CAP    = (BLOCK_HEIGHT*4)+PLATE_HEIGHT+ADJUSTED;

    public Lift(){
    }
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLift  = hwMap.get(DcMotor.class, "front_lift");
        backLift   = hwMap.get(DcMotor.class, "back_lift");

        //Directions subject to change when motor facing is identified
        frontLift.setDirection(DcMotor.Direction.REVERSE);
        backLift.setDirection(DcMotor.Direction.REVERSE);

        frontLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        frontLift.setPower(0);
        backLift.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define and initialize ALL installed sensors
        REVTouchBottom = hwMap.get(DigitalChannel.class, "Bottom_Touch");

        // set the digital channel to input.
        REVTouchBottom.setMode(DigitalChannel.Mode.INPUT);
    }
    //control the power of the lift motors
    public void liftPower (double power){
        frontLift.setPower(-power);
        backLift.setPower(power);
    }
    //send the lift down to its lowest height
    public void liftDown (double power) {
        // if the digital channel returns true it's HIGH and the button is unpressed.
        if (REVTouchBottom.getState()) {
            frontLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            liftPower(power);//May need to be fixed for direction
            while (REVTouchBottom.getState());
            liftPower(0);

            frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    //NOTE: ONE is ground level
    public void placeLevel (PlaceLevel level) {
        switch (level){
            case ONE:
                liftDown(-SPEED);

                break;
            case INSIDE:
                frontLift.setTargetPosition(LEVEL_INSIDE);
                backLift.setTargetPosition(LEVEL_INSIDE);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower(SPEED);
                while(frontLift.isBusy() && backLift.isBusy());
                liftPower(0);
                frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case TWO:
                frontLift.setTargetPosition(LEVEL_TWO);
                backLift.setTargetPosition(LEVEL_TWO);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower(SPEED);
                while(frontLift.isBusy() && backLift.isBusy());
                liftPower(0);
                frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case THREE:
                frontLift.setTargetPosition(LEVEL_THREE);
                backLift.setTargetPosition(LEVEL_THREE);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower(SPEED);
                while(frontLift.isBusy() && backLift.isBusy());
                liftPower(0);
                frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case FOUR:
                frontLift.setTargetPosition(LEVEL_FOUR);
                backLift.setTargetPosition(LEVEL_FOUR);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower(SPEED);
                while(frontLift.isBusy() && backLift.isBusy());
                liftPower(0);
                frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case FIVE:
                frontLift.setTargetPosition(LEVEL_FIVE);
                backLift.setTargetPosition(LEVEL_FIVE);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower(SPEED);
                while(frontLift.isBusy() && backLift.isBusy());
                liftPower(0);
                frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            case CAP:
                frontLift.setTargetPosition(LEVEL_CAP);
                backLift.setTargetPosition(LEVEL_CAP);
                frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPower(SPEED);
                while(frontLift.isBusy() && backLift.isBusy());
                liftPower(0);
                frontLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
        }
    }
    public double getFrontLiftEncoder(){return frontLift.getCurrentPosition();}
    public double getBackLiftEncoder(){return backLift.getCurrentPosition();}
    public int getLiftPosition() {
        return Math.round(frontLift.getCurrentPosition()/BLOCK_HEIGHT);
    }

}
*/