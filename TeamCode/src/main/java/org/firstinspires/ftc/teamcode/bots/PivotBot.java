package org.firstinspires.ftc.teamcode.bots;

        import com.acmerobotics.dashboard.config.Config;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorImplEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.stormbots.MiniPID;

        import java.util.Timer;
        import java.util.TimerTask;
@Config
public class PivotBot extends OdometryBot {

    public static int samplePivotDropOffPos = 1900;
    public static int maximumPivotPos = 1300;
    public static int minumumPivotPos = -100;

    public static int sampleSlideDropOffPos = 2000;

    public static int maximumSlidePos = 2600;
    public static int minimumSlidePos = 0;

    public boolean pivotOutOfRange = false;
    public int pivotTarget = 20;
    public double pivotPower = 0;
    public int slideTarget = 20;

    public DcMotorEx pivotMotor = null;
    public DcMotorEx pivotMotor1 = null;
    public DcMotorEx slideMotor1 = null;
    public DcMotorEx slideMotor = null;




    public static int searchPivotPos = 220; // tested
    public static int pickupSpecimenPivotPos = 115; // tested
    public static int pickupSamplePivotPos = 20; // tested
    public static int pickupUpPivotPos = 400;
    public static int specimenHighPivotPos = 1150;
    public static int specimenLowPivotPos = 750;
    public static int highBasketPivotPos = 1200;
    public static int lowBasketPivotPos = 1000;
    // Pivot timmer
    private Timer pivotTimer = new Timer();
    private TimerTask pivotTimerTask;
    public static int searchSlidePos = 350;
    public static int specimenHighSlidePos = 716;
    public static int specimenLowSlidePos = 800;
    public static int highBasketSlidePos = 2540;
    public static int lowBasketSlidePos = 800;
    public static double slideCMCoefficient = 31.2;




    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        pivotMotor1 = hwMap.get(DcMotorEx.class, "pivot2");
        pivotMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        pivotMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor1.setPower(pivotPower);

        pivotMotor = hwMap.get(DcMotorEx.class, "pivot1");
        pivotMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setPower(pivotPower);

        slideMotor1 = hwMap.get(DcMotorEx.class, "slide1");
        slideMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor1.setPower(0);

        slideMotor = hwMap.get(DcMotorEx.class, "slide2");
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setPower(0);


        // TODO
        pivotMotor1.setTargetPosition(pivotTarget);
        pivotMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setTargetPosition(pivotTarget);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotPower = 0;
    }

    public PivotBot(LinearOpMode opMode) {
        super(opMode);
    }

    public int getSlidePosition() {
        return slideMotor1.getCurrentPosition();
    }

    public int getPivotPosition() {
        return pivotMotor1.getCurrentPosition();
    }

    protected void onTick() {

        super.onTick();
//
//        if (pivotTarget > minumimPivotPos - 100 && pivotTarget < maximumPivotPos + 100){
//
//            pivotOutOfRange = false;
//
//            pivotMotor1.setTargetPosition(pivotTarget);
//            pivotMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            pivotMotor.setTargetPosition(pivotTarget);
//            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//            // TODO : PID control for the pivot motor
//            pivotMotor1.setPower(0.6);
//            pivotMotor.setPower(0.6);
//
//        } else {
//
//            pivotOutOfRange = true;
//            pivotMotor1.setPower(0);
//            pivotMotor.setPower(0);
//
//        }
//        if (slideTarget > 0 && slideTarget < maximumSlidePos + 100){
//
//            slideMotor1.setTargetPosition(slideTarget);
//            slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            slideMotor.setTargetPosition(slideTarget);
//            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // TODO : PID control for the slide motor
//            slideMotor1.setPower(0.5);
//            slideMotor.setPower(0.5);
//
//        } else {
//
//            slideMotor1.setPower(0);
//            slideMotor.setPower(0);
//
//        }
    }
    public void slideByDelta(int delta){
        slideTarget += delta;
    }
    public void slideRunToPosition(int slideTarget)
    {
        slideMotor1.setTargetPosition(slideTarget);
        slideMotor.setTargetPosition(slideTarget);

    }
    public void slideControl(boolean up, boolean down) {
        if (up) {
            if (slideMotor1.getCurrentPosition() < maximumSlidePos) {
                slideTarget = slideMotor1.getCurrentPosition() + ((maximumSlidePos - slideMotor1.getCurrentPosition()) / 10);
                slideMotor1.setTargetPosition(slideTarget);
                slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setTargetPosition(slideTarget);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }
        if (down) {
            if (slideMotor1.getCurrentPosition() > minimumSlidePos) {
                slideTarget = slideMotor1.getCurrentPosition() - (slideMotor1.getCurrentPosition() / 10);
                slideMotor1.setTargetPosition(slideTarget);
                slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setTargetPosition(slideTarget);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }
        }

        //make pivot same
    }

    public void pivotRunToPosition (int target){
        pivotTarget = target;
    }


//    public void pivotControl(boolean up, boolean down){
//        if (up) {
//            if (pivotMotor1.getCurrentPosition() < maximumPivotPos) {
//                pivotTarget = pivotMotor1.getCurrentPosition() + ((maximumPivotPos - pivotMotor1.getCurrentPosition()) / 10);
//                pivotMotor1.setTargetPosition(pivotTarget);
//                pivotMotor.setTargetPosition(pivotTarget);
//                pivotMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//        }
//        if (down) {
//            if (pivotMotor1.getCurrentPosition() > minumimPivotPos) {
//                pivotTarget = pivotMotor1.getCurrentPosition() - (pivotMotor1.getCurrentPosition() / 10);
//                pivotMotor1.setTargetPosition(pivotTarget);
//                pivotMotor.setTargetPosition(pivotTarget);
//                pivotMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//        }
//    }
    public void relatePivotToSlide(){
        pivotTarget = Math.round((slideMotor1.getCurrentPosition() / -23) + 245);
        pivotMotor1.setTargetPosition(pivotTarget);
        pivotMotor.setTargetPosition(pivotTarget);
        pivotMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




    public void pivotTo(int pos){
        pivotTarget = pos;
    }

    public void pivotToSearchPos(){
        pivotTarget = searchPivotPos;
    }
    //    public void pivotToPickupPos(boolean isSpecimen){
//        if (isSpecimen){
//            pivotTarget = pickupSpecimenPivotPos;
//        } else {
//            pivotTarget = pickupSamplePivotPos;
//        }
//    }
    public void pivotToPickupPosSpecimen(){
        pivotTarget = pickupSpecimenPivotPos;
    }
    public void pivotToPickupPosSample(){
        pivotTarget = pickupSamplePivotPos;
    }
    public void pivotToPickupUpPos(){
        //pivotTarget = pickupUpPivotPos;
        relatePivotToSlide();
    }
    public void pivotToSpecimenHighPos(){
        pivotTarget = specimenHighPivotPos;
    }
    public void pivotToSpecimenLowPos(){
        pivotTarget = specimenLowPivotPos;
    }
    public void pivotToHighBasketPos(){
        pivotTarget = highBasketPivotPos;
    }
    public void pivotToLowBasketPos(){
        pivotTarget = lowBasketPivotPos;
    }
    public void pivotByDelta(int delta){
        pivotTarget += delta;
    }

    public void cancelPivotTimer(){
        pivotTimer.cancel();
    }

    public void moveSlide(int pos){
        slideTarget = pos;
    }

    public void moveSlideToSearchPos(){
        slideTarget = searchSlidePos;
    }
    public void moveSlideToHighSpecimenPos(){
        slideTarget = specimenHighSlidePos;
    }
    public void moveSlideToLowSpecimenPos(){
        slideTarget = specimenLowSlidePos;
    }
    public void moveSlideByDelta(int delta){
        slideTarget += delta;
    }

    /**
     * Move the slide by a delta value in CM
     * @param delta
     */
    public void moveSlideByDelta(double delta){
        slideTarget += Math.round(delta * slideCMCoefficient);
    }
    public void moveSlideToHighBucketPos(){
        slideTarget = highBasketSlidePos;
    }
    public void moveSlideToLowBucketPos(){
        slideTarget = lowBasketSlidePos;
    }

}