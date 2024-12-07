package org.firstinspires.ftc.teamcode.bots;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import java.util.Timer;
        import java.util.TimerTask;

public class PivotBot extends OdometryBot {

    ElapsedTime timer = new ElapsedTime();


    // Pivot motor constants
    public final int maximumPivotPos = 1300;
    public final int minumimPivotPos = -100;
    private final int searchPivotPos = 250; // tested
    private final int pickupSpecimenPivotPos = 130; // tested
    private final int pickupSamplePivotPos = 120; // tested
    private final int pickupUpPivotPos = 400;
    public final int specimenPivotPos = 900;
    public final int highBucketPivotPos = 1200;
    public boolean pivotOutOfRange = false;
    public int pivotTarget = 100;
    public double pivotPower = 0.7;
    // Pivot timmer
    private Timer pivotTimer = new Timer();
    private TimerTask pivotTimerTask;

    // Slide motor constants
    public final int maximumSlidePos = 2400;
    public final int searchSlidePos = 350;
    public final int specimenSlidePos = 1000;
    public final int highBucketSlidePos = 1500;
    public final int lowBucketSlidePos = 800;

    public int slideTarget = 110;


    public DcMotorEx pivotMotor = null;
    public DcMotorEx slideMotor = null;

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

        // TODO
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

        if (pivotTarget > minumimPivotPos - 100 && pivotTarget < maximumPivotPos + 100){

            pivotOutOfRange = false;

            pivotMotor.setTargetPosition(pivotTarget);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // TODO : PID control for the pivot motor
            pivotMotor.setPower(0.6);

        } else {

            pivotOutOfRange = true;
            pivotMotor.setPower(0);

        }
        if (slideTarget > 0 && slideTarget < maximumSlidePos + 100){

            slideMotor.setTargetPosition(slideTarget);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // TODO : PID control for the slide motor
            slideMotor.setPower(0.5);

        } else {

            slideMotor.setPower(0);

        }
    }
    public void slideByDelta(int delta){
        slideTarget += delta;
    }
    public void slideControl(boolean up, boolean down) {
        if (up) {
            if (slideMotor.getCurrentPosition() < maximumSlidePos) {
                slideTarget = slideMotor.getCurrentPosition() + ((maximumSlidePos - slideMotor.getCurrentPosition()) / 10);
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

        //make pivot same
    }

    public void pivotControl(boolean up, boolean down){
        if (up) {
            if (pivotMotor.getCurrentPosition() < maximumPivotPos) {
                pivotTarget = pivotMotor.getCurrentPosition() + ((maximumPivotPos - pivotMotor.getCurrentPosition()) / 10);
                pivotMotor.setTargetPosition(pivotTarget);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        if (down) {
            if (pivotMotor.getCurrentPosition() > minumimPivotPos) {
                pivotTarget = pivotMotor.getCurrentPosition() - (pivotMotor.getCurrentPosition() / 10);
                pivotMotor.setTargetPosition(pivotTarget);
                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
    }

    public void pivotTo(int pos, @Deprecated double power){
        pivotTarget = pos;
    }
    public void pivotToSearchPos(){
        pivotTarget = searchPivotPos;
    }
    public void pivotToPickupPos(boolean isSpecimen){
        if (isSpecimen){
            pivotTarget = pickupSpecimenPivotPos;
        } else {
            pivotTarget = pickupSamplePivotPos;
        }
    }
    public void pivotToPickupUpPos(){
        pivotTarget = pickupUpPivotPos;
    }
    public void pivotToSpecimenPos(){
        pivotTarget = specimenPivotPos;
    }
    public void pivotToHighBucketPos(){
        pivotTarget = highBucketPivotPos;
    }

    public void pivotToUpPosInTime(int time){
        pivotTimerTask = new TimerTask() {
            @Override
            public void run() {
                pivotToPickupUpPos();
            }
        };
        pivotTimer.schedule(pivotTimerTask, time);
    }
    public void cancelPivotTimer(){
        pivotTimer.cancel();
    }

    public void moveSlide(int pos, @Deprecated double power){
        slideTarget = pos;
    }

    public void moveSlideToSearchPos(){
        slideTarget = searchSlidePos;
    }
    public void moveSlideToSpecimenPos(){
        slideTarget = specimenSlidePos;
    }
    public void moveSlideToHighBucketPos(){
        slideTarget = highBucketSlidePos;
    }
    public void moveSlideToLowBucketPos(){
        slideTarget = lowBucketSlidePos;
    }

    public void relatePivotToSlide(){
        pivotTarget = Math.round((slideMotor.getCurrentPosition() / -23) + 245);
        pivotMotor.setTargetPosition(pivotTarget);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}