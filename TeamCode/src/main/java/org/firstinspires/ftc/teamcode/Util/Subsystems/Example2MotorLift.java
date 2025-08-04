package org.firstinspires.ftc.teamcode.Util.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Util.Names;
import org.firstinspires.ftc.teamcode.Util.PDFLController;



@Config
@Disabled
public class Example2MotorLift extends SubsystemBase {

    private final DcMotorEx liftLeft;
    private final DcMotorEx liftRight;

    public static int liftTarget = 0;

    public static int P = 0, D = 0, F = 0, L = 0;

    public Names.EncoderReadingFrom readingFrom = Names.EncoderReadingFrom.BOTH ;

    PDFLController pdflController;
    public static double power;

    public static int allowedError = 1;

    private double currentPosition;



    public Example2MotorLift(HardwareMap hardwareMap) {
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");

        liftLeft.setDirection(DcMotorEx.Direction.REVERSE);
        liftRight.setDirection(DcMotorEx.Direction.FORWARD);

        resetMotors();

    }

    public void setLiftTarget(int target){
        liftTarget = target;
    }

    public void setAllowedError(int error){
        allowedError = error;
    }

    public void resetMotors(){
        liftLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(){

        currentPosition = getCurrentPosition();

        pdflController.setPDFL(P, D, F, L);
        pdflController.setTarget(liftTarget);
        pdflController.update(currentPosition);



        power = pdflController.runPDFL(allowedError);
        liftLeft.setPower(power);
        liftRight.setPower(power);
    }

    public void debugTelemetry(Telemetry telemetry){
        telemetry.addData("Power Supplied ", power);
        telemetry.addData("Left Encoder ", liftLeft.getCurrentPosition());
        telemetry.addData("Right Encoder ", liftRight.getCurrentPosition());
        telemetry.addData("Target ", liftTarget);
        telemetry.addData("Reading From ", readingFrom);
        telemetry.addData("Current Left (MA) ", liftLeft.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Current Right (MA) ", liftRight.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Current Read Position ", getCurrentPosition());

    }

    public boolean isAtTarget(){
        return Math.abs(liftTarget - currentPosition) < allowedError;
    }

    public int getCurrentPosition(){
        return readingFrom == Names.EncoderReadingFrom.LEFT ? liftLeft.getCurrentPosition()   :
                (int) (readingFrom == Names.EncoderReadingFrom.RIGHT ? liftRight.getCurrentPosition() :
                        (double) (liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2);
    }


}




