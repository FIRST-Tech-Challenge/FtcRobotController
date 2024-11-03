package org.firstinspires.ftc.teamcode.bots;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotBot extends LimelightBot {

    public int slideTarget = 0;
    public int slideTarget2 = 0;
    public DcMotorEx pivotMotor = null;
    public DcMotorEx slideMotor = null;
    
    public int pivotPosition = 0;

    private int slidePower = 200;

    private final int limitMax = 200;

    public boolean isDown = true;
    protected boolean isEndOfAuto = false;

    public final double endPosition = 300;

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        pivotMotor = hwMap.get(DcMotorEx.class, "Pivot Motor");
        pivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(0);

        slideMotor = hwMap.get(DcMotorEx.class, "slide");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPower(0);
    }

    public PivotBot(LinearOpMode opMode) {
        super(opMode);
    }

    public int getSlidePosition() {
        return pivotMotor.getCurrentPosition();
    }

    protected void onTick() {
        super.onTick();
        pivotMotor.setTargetPosition(slideTarget);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(0.3);

        slideMotor.setTargetPosition(slideTarget2);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.5);
    }

    public void slideControl(boolean up, boolean down) {
        if (up) {
//            slideTarget2 = 20;
            if (slideMotor.getCurrentPosition() < 2000) {
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + 20);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (down) {
//            slideTarget2 = 2000;
                if (slideMotor.getCurrentPosition() > 20)
                    slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - 20);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
//        if (Math.abs(Slide.getCurrentPosition() - slideTarget2) < 3){
//            Slide.setPower(0);
//        }
        }
    }

        public void slideMove(int pos){
            //depending on the number, move the slide
            slideMotor.setTargetPosition(pos);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void pivotControl(boolean up, boolean down){
            if (up) {
                slideTarget = 0;
                pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition() + 20);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (down) {
                slideTarget = 900;
                pivotMotor.setTargetPosition(pivotMotor.getCurrentPosition() - 20);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
//        if (Math.abs(Motor.getCurrentPosition() - slideTarget) < 3){
//            Motor.setPower(0);
//        }
        }

        public void pivotTo(int pos){
            pivotMotor.setTargetPosition(pos);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



    }


