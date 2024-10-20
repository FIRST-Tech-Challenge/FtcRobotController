package org.firstinspires.ftc.teamcode.bots;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotBot extends GyroBot{

    public int slideTarget = 0;
    public DcMotorEx Motor = null;
    public int pivotPosition = 0;

    private int slidePower = 200;

    private final int limitMax = 200;

    public boolean isDown = true;
    protected boolean isEndOfAuto = false;

    public final double endPosition = 300;
    @Override
    public void init(HardwareMap ahwMap){
        super.init(ahwMap);
        Motor = hwMap.get(DcMotorEx.class, "Pivot Motor");
        Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Motor.setPower(0);


    }

    public PivotBot(LinearOpMode opMode) {super(opMode);}

    public int getSlidePosition(){
        return Motor.getCurrentPosition();
    }

//    public void slideDown(boolean button) {
//        if (button && slidePosition>0){
//            rightMotor.setPower(-1);
//            leftMotor.setPower(1);
//            slidePosition--;
//        } else {
//            rightMotor.setPower(0);
//            leftMotor.setPower(0);
//        }
//    }

//    public void slideUp(boolean button) {
//        if (button && slidePosition<endPosition){
//            rightMotor.setPower(1);
//            leftMotor.setPower(-1);
//            slidePosition++;
//        } else {
//            rightMotor.setPower(0);
//            leftMotor.setPower(0);
//        }
//    }

    protected void onTick() {
        super.onTick();
        Motor.setTargetPosition(slideTarget);
        Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor.setPower(0.3);
    }

    public void PivotControl(boolean up, boolean down){
        if (up){
            slideTarget = 0;
            Motor.setTargetPosition(slideTarget);
            Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(down){
            slideTarget = 900;
            Motor.setTargetPosition(slideTarget);
            Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Math.abs(Motor.getCurrentPosition() - slideTarget) < 3){
            Motor.setPower(0);
        }
    }



}