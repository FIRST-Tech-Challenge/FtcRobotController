package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Double.isNaN;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Scheduler;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Subsystem;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.ButtonEX;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.ContinuousInput;
import org.firstinspires.ftc.teamcode.teamUtil.ConfigNames;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

public class Lift extends Subsystem {
    RobotConfig r;

    private static DcMotor lift0;
    private static DcMotor lift1;
    private static double liftPos;

    private static double previousTime = 0; //needs to be given a value of 0 at the start
    private static double previousError = 0; //needs to be given a value of 0 at the start
    private static double I = 0; //needs to be given a value of 0 at the start
    private static double targetPos = 0; //needs to be set whenever you need it to be changed
    private static double spoolHoldPosition = 0;
    private static boolean spoolHold = false;

    ContinuousInput liftPositioner;
    ButtonEX limitIsPressed;
    RobotConstants.poleHeights targetHeight;


    public Lift(RobotConfig r) {
        this.r = r;
    }
    public Lift(){
        this(RobotConfig.getInstance());
    }

    private void resetEncoders(){
        lift0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init() {
        lift0 = r.opMode.hardwareMap.get(DcMotor.class, ConfigNames.lift0);
        lift1 = r.opMode.hardwareMap.get(DcMotor.class, ConfigNames.lift1);
        previousTime = 0;
        previousError = 0;
        targetPos = 0;
        spoolHoldPosition = 0;
        spoolHold = false;
    }

    @Override
    public void read() {
        double lift0Pos = lift0.getCurrentPosition();
        double lift1Pos = lift1.getCurrentPosition();

        if(((lift0Pos + lift1Pos) / 2)<-1000) liftPos = Math.min(lift0Pos, lift1Pos);
        else if(((lift0Pos + lift1Pos) / 2)>=-1000) liftPos = Math.max(lift0Pos, lift1Pos);
        else liftPos = 0;
    }

    @Override
    public void update(){

        //needs to be set to runtime.time()
        double currentTime = RobotConfig.elapsedTime.time();

        positionControl(liftPositioner.value(), limitIsPressed.isPressed(), targetHeight);

        if(limitIsPressed.onPress()){
            resetEncoders();
        }

        double currentError = (targetPos - liftPos);
        final double kP = 0.003;
        final double kI = 0.07;
        final double kD = 0.0003;

        double P = kP*currentError;
        I = I + (kI*(currentError*(currentTime -previousTime)));
        if (I < -0.5) {
            I = -0.5;
        }
        else if (I > 0){
            I = 0;
        }

        double D = kD*((currentError-previousError)/(currentTime -previousTime));

        double finalPower = (P + I + D);
        lift0.setPower(finalPower);
        lift1.setPower(finalPower);

        previousError = currentError;
        previousTime = currentTime;
    }

    @Override
    public void close() {

    }

    private void positionControl(double liftPosition, boolean limitIsPressed, RobotConstants.poleHeights poleHeight){
        Lift.targetPos -= ((int) (liftPosition*100));
        if (isNaN(liftPosition) && !spoolHold){
            spoolHoldPosition = Lift.liftPos;
            spoolHold = true;
        }
        if(isNaN(liftPosition)){
            Lift.targetPos = spoolHoldPosition;
        }
        if (!isNaN(liftPosition)){
            spoolHold = false;
        }
        if((Lift.targetPos > Lift.liftPos) && limitIsPressed){
            Lift.targetPos = Lift.liftPos;
        }
        if (Lift.targetPos>0){
            Lift.targetPos = 0;
        }
        if(Lift.targetPos < (RobotConstants.poleHeights.HIGH.getEncoderValue() - 100)) {
            Lift.targetPos = (RobotConstants.poleHeights.HIGH.getEncoderValue() - 100);
        }
        switch (poleHeight){
            case HIGH:
                Lift.targetPos = RobotConstants.poleHeights.HIGH.getEncoderValue();
                spoolHoldPosition = RobotConstants.poleHeights.HIGH_DROP.getEncoderValue();
                break;
            case MEDIUM:
                Lift.targetPos = RobotConstants.poleHeights.MEDIUM.getEncoderValue();
                spoolHoldPosition = RobotConstants.poleHeights.MEDIUM_DROP.getEncoderValue();
                break;
            case LOW:
                Lift.targetPos = RobotConstants.poleHeights.LOW.getEncoderValue();
                spoolHoldPosition = RobotConstants.poleHeights.LOW_DROP.getEncoderValue();
                break;
            case GROUND:
                Lift.targetPos = RobotConstants.poleHeights.GROUND.getEncoderValue();
                spoolHoldPosition = RobotConstants.poleHeights.GROUND.getEncoderValue();
                break;
            default:
                break;
        }
    }

    private RobotConstants.poleHeights buttonAnalysis(ButtonEX[] top_middle_low_ground){
        if (top_middle_low_ground[0].isPressed()) return RobotConstants.poleHeights.HIGH;
        if (top_middle_low_ground[1].isPressed()) return RobotConstants.poleHeights.MEDIUM;
        if (top_middle_low_ground[2].isPressed()) return RobotConstants.poleHeights.LOW;
        if (top_middle_low_ground[3].isPressed()) return RobotConstants.poleHeights.GROUND;
        return RobotConstants.poleHeights.IDLE;
    }

    public void liftInputs(ContinuousInput liftPositionInput, ButtonEX limitSwitch, ButtonEX[] top_middle_low_ground){
        liftPositioner = liftPositionInput;
        limitIsPressed = limitSwitch;
        targetHeight = buttonAnalysis(top_middle_low_ground);
    }
}
