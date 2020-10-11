package org.firstinspires.ftc.teamcode.robots.icarus;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Abhijit on 11/7/18.
 */
public class Superman {
    DcMotor supermanLeft = null;
    DcMotor supermanRight = null;
    double supermanPwr = 0;

    int supermanPosInternal = 0;
    int supermanPos = 0;

    //Positions for moving supermanLeft during articulations
    public int pos_Intake;
    public int pos_reverseIntake;
    public int pos_reverseDeposit;
    public int pos_Deposit;
    public int pos_DepositPartial;
    public int pos_Maximum;
    public int pos_autonPrelatch;
    public int pos_prelatch;
    public int pos_latched;
    public int pos_postlatch;
    public int pos_stowed;
    public int pos_driving; //todo - experiment with driving with supermanLeft set around 100 (slightly angled) to see if it is more responsive - higher battery drain because supermanLeft is straining, but less actual downforce on omni
    public int pos_tipped;


    //filler value; needs to be updated to reflect actual ratio
    public double ticksPerDegree = 6.44705882;//23.64444444444

    public double multiplier = 23.64444444444444444444444444444444444444444/ 6.44705882;

    public boolean active = true;




    public Superman(PoseBigWheel.RobotType currentBot, DcMotor supermanLeft, DcMotor supermanRight) {
        supermanLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        supermanRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.supermanLeft = supermanLeft;
        this.supermanRight = supermanRight;

        switch (currentBot){
            case BigWheel:
                supermanLeft.setDirection(DcMotor.Direction.REVERSE);
                //supermanRight.setDirection(DcMotorSimple.Direction.REVERSE);

                pos_Intake = 10;
                pos_reverseIntake = 452;
                pos_reverseDeposit = 343;
                pos_Deposit = 354;
                pos_DepositPartial = 200;
                pos_Maximum = 500;
                pos_autonPrelatch = 400;
                pos_prelatch = 500;
                pos_latched = 500;
                pos_postlatch = 0;
                pos_stowed = 0;
                pos_driving = 0;
                pos_tipped = 2142;
                break;
            case Icarus:
                supermanLeft.setDirection(DcMotor.Direction.REVERSE);
                //supermanRight.setDirection(DcMotorSimple.Direction.REVERSE);

                pos_Intake = (int) (multiplier*10);
                pos_reverseIntake = (int) (multiplier*360);
                pos_reverseDeposit =  880;
                pos_Deposit = 880;
                pos_DepositPartial = 880;
                pos_autonPrelatch = 1390;
                pos_prelatch = 1390; //(int) (multiplier*335);
                pos_latched = 1370;
                pos_postlatch = 0;
                pos_stowed = 0;
                pos_driving = 0;
                pos_tipped = 2142;
                pos_Maximum = pos_tipped;
                break;
        }
    }





    public void update() {
        if (active && supermanPosInternal != supermanPos) { //don't keep updating if we are retractBelt to target position
            supermanPosInternal = supermanPos;

            supermanLeft.setTargetPosition(supermanPos);
            supermanLeft.setPower(supermanPwr);

            supermanRight.setTargetPosition(supermanPos);
            supermanRight.setPower(supermanPwr);
        }
    }
    public void kill() {
        setPower(0);
        update();
        active = false;
    }

    public void restart(double pwr) {
        setPower(pwr);
        active = true;
    }
    public void resetEncoders() {
        //just encoder - only safe to call if we know robot is in normal ground state
        //this should stop the motor
        //todo: maybe verify against imu - though that would assume that imu was calibrated correctly
        supermanLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        supermanLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        supermanRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        supermanRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public boolean reset(){
        //reset the supermanLeft arm by retracting it to the physical start position on very low power until it stops moving
        //this has to be called repeatedly until it returns true - unless we decide to start a dedicated thread
        //only safe to call if supermanLeft is free to move and is not hyper-extended - operator needs to verify this
        //todo: this is just a stub - need to decide if we need to implement

        return false;
        // start tucking in the arm
        //            supermanLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //            supermanLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //            supermanLeft.setPower(.1); // we assume this is low enough to move it until it jams without skipping gears

        //after stalling
        //            supermanLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //            supermanLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //            setTargetPosition (probably back to zero or minimum) - but this makes position control active again
    }

    public boolean isActive() {
        return active;
    }

    public void setPower(double pwr) {
        supermanPwr = pwr;
    }
    public void setTargetPosition(int pos) {
        supermanPos = pos;
    }
    public boolean setTargetPosition(int pos, double speed) {
        setPower(speed);
        setTargetPosition(pos);
        if (nearTarget()) return true;
        else return false;
    }

    public int getTargetPosition() {
        return supermanPos;
    }
    public int getCurrentPosition() {
        return supermanLeft.getCurrentPosition();
    }
    public int getCurrentPosition2() {
        return supermanRight.getCurrentPosition();
    }

    public double getCurrentAngle(){return  supermanLeft.getCurrentPosition()/ticksPerDegree;}
    public boolean nearTarget(){
        if ((Math.abs(getCurrentPosition()-getTargetPosition()))<15) return true;
        else return false;
    }


    public void raise() {
        setTargetPosition(Math.min(getCurrentPosition() + 100, 4000));
    }
    public void lower() {
        setTargetPosition(Math.max(getCurrentPosition() - 100 , 0));
    }
    public void runToAngle(double angle) {
        setTargetPosition((int) (angle * ticksPerDegree));
    }//untested

}
