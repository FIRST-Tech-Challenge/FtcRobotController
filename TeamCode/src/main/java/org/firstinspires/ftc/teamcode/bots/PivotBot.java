package org.firstinspires.ftc.teamcode.bots;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotBot extends LimelightBot {

    private int maximumPivot = 1300;
    private int minumimPivot = -100;
    public boolean pivotOutOfRange = false;

    public int slideTarget = 110;
    public int pivotTarget = 100;

    private double pivotPower = 0.5;

    private DcMotorEx pivotMotor = null;
    private DcMotorEx slideMotor = null;

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        pivotMotor = hwMap.get(DcMotorEx.class, "Pivot Motor");
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(pivotPower);

        slideMotor = hwMap.get(DcMotorEx.class, "slide");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPower(0);

        pivotMotor.setTargetPosition(pivotTarget);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotPower = 0.5;
    }

    public PivotBot(LinearOpMode opMode) {
        super(opMode);
    }

    public int getSlidePosition() {
        return slideMotor.getCurrentPosition();
    }

    public int getPivotPosition() {
        return pivotMotor.getCurrentPosition();
    }

    protected void onTick() {
        super.onTick();

        if (pivotTarget > minumimPivot - 100 && pivotTarget < maximumPivot + 100){

            pivotOutOfRange = false;

            pivotMotor.setTargetPosition(pivotTarget);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            pivotPower = 0.3 + ((Math.abs(pivotTarget - pivotMotor.getCurrentPosition()) / maximumPivot) * 0.5);
            pivotMotor.setPower(0.6);

        } else {

            pivotOutOfRange = true;
            pivotMotor.setPower(0);

        }

        slideMotor.setTargetPosition(slideTarget);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.5);
    }

    public void slideControl(boolean up, boolean down) {
        if (up) {
            if (slideMotor.getCurrentPosition() < 1950) {
                slideTarget = slideMotor.getCurrentPosition() + ((1950 - slideMotor.getCurrentPosition()) / 10);
                slideMotor.setTargetPosition(slideTarget);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        if (down) {
            if (slideMotor.getCurrentPosition() > 170) {
                slideTarget = slideMotor.getCurrentPosition() - (slideMotor.getCurrentPosition() / 10);
                slideMotor.setTargetPosition(slideTarget);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

    public void pivotControl(boolean up, boolean down){
//        if (slideMotor.getCurrentPosition() < 200) {
//            if (up) {
//                pivotTarget = maximumPivot;
//                pivotPower = 0.6;
//            }
//            if (down) {
//                pivotTarget = minumimPivot;
//                pivotPower = 0.6;
//            }
//        }
        if (up) {
            if (pivotMotor.getCurrentPosition() < maximumPivot) {
                pivotTarget = pivotMotor.getCurrentPosition() + ((maximumPivot - pivotMotor.getCurrentPosition()) / 10);
                pivotMotor.setTargetPosition(pivotTarget);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        if (down) {
            if (pivotMotor.getCurrentPosition() > minumimPivot) {
                pivotTarget = pivotMotor.getCurrentPosition() - (pivotMotor.getCurrentPosition() / 10);
                pivotMotor.setTargetPosition(pivotTarget);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

    public void pivotTo(int pos){
        pivotMotor.setTargetPosition(pos);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveSlide(int pos){
        slideMotor.setTargetPosition(pos);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    }


