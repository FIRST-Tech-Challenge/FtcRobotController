package org.firstinspires.ftc.teamcode.bots;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;

public class PivotBot extends LimelightBot{

    public int slideTarget = 0;
    public int slideTarget2 = 0;
    public DcMotorEx Motor = null;
    public DcMotorEx Slide = null;
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

        Slide = hwMap.get(DcMotorEx.class, "slide");
        Slide.setDirection(DcMotorSimple.Direction.REVERSE);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setPower(0);
    }

    public PivotBot(LinearOpMode opMode) {super(opMode);}

    public int getSlidePosition(){
        return Motor.getCurrentPosition();
    }

    protected void onTick() {
        super.onTick();
        Motor.setTargetPosition(slideTarget);
        Motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Motor.setPower(0.3);

        Slide.setTargetPosition(slideTarget2);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(0.5);
    }

    public void slideControl(boolean up, boolean down){
        if (up){
            slideTarget2 = 20;
            Slide.setTargetPosition(slideTarget2);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(down){
            slideTarget2 = 2000;
            Slide.setTargetPosition(slideTarget2);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (Math.abs(Slide.getCurrentPosition() - slideTarget2) < 3){
            Slide.setPower(0);
        }
    }

    public void slideMove(int up){
        //depending on the number, move the slide
    }

    public void pivotControl(boolean up, boolean down){
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