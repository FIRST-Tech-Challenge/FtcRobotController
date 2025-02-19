//package org.firstinspires.ftc.teamcode.bots;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//
//
//public class AutomationBot extends LimelightBot{
//    private boolean slideInSpecimenPosition = false;
//    private boolean slideInHighBasketPosition = false;
//    private boolean pivotInSpecimenPosition = false;
//    private boolean pivotInHighBasketPosition = false;
//
//    public AutomationBot(LinearOpMode opMode) {
//        super(opMode);
//    }
//
//
//    public void init(HardwareMap hardwareMap){
//        super.init(hardwareMap);
//
//
//    }
//
//    protected void onTick() {
//        super.onTick();
////        pivotInSpecimenPosition = (getPivotPosition() >= specimenHighPivotPos - 10) && (getPivotPosition() <= specimenHighPivotPos + 10);
////        slideInSpecimenPosition = (getSlidePosition() >= specimenHighSlidePos - 10) && (getSlidePosition() <= specimenHighSlidePos + 10);
////        pivotInHighBasketPosition = (getPivotPosition() >= highBasketPivotPos - 10) && (getPivotPosition() <= highBasketPivotPos + 10);
////        slideInHighBasketPosition = (getSlidePosition() >= highBasketSlidePos - 10) && (getSlidePosition() <= highBasketSlidePos + 10);
//    }
//
//    public void goToDefaultPosition(boolean input){
//        if (input) {
////            moveSlide(minimumSlidePos);
////            schedule(this::relatePivotToSlide, 1500);
//        }
//    }
//    public void scoreSpecimen(boolean input) {
//        if (input) {
//            if (!pivotInSpecimenPosition || !slideInSpecimenPosition) {
//
////                pivotToSpecimenHighPos();
////                rotateToVerticalPos();
////                schedule(this::moveSlideToHighSpecimenPos, 600);
//            }
//            else{
////                moveSlideByDelta(-350);
////                schedule(this::openPinch, 600);
//
//            }
//        }
//    }
//    public void scoreBucket(boolean input) {
//        if (input) {
//            if (!pivotInHighBasketPosition || !slideInHighBasketPosition) {
////                pivotToHighBasketPos();
////                schedule(this::moveSlideToHighBucketPos, 1000);
//            }
//            else{
//                openPinch();
//            }
//        }
//    }
//
//    /**
//     * This method will prepare the robot to score a specimen (raise pivot arm, extend slide to a certain position)
//     * @param input - boolean to determine if the robot should prepare to score a specimen
//     * @param aimHigh - boolean to determine if the robot should aim high or low chamber
//     */
//    public void readySpecimenPos(boolean input, boolean aimHigh) {
//        if (input) {
//            rotateToVerticalPos();
//            if (aimHigh) {
////                pivotToSpecimenHighPos();
////                schedule(this::moveSlideToHighSpecimenPos, 300);
//            } else {
////                pivotToSpecimenLowPos();
////                moveSlideToLowSpecimenPos();
//            }
//        }
//    }
//
//    /**
//     * This method will score a specimen (slide down a little, open pinch)
//     * @param input
//     */
////    public void scoreSpecimenSimple(boolean input) {
////        if (input) {
////            moveSlideByDelta(-180);
////            schedule(this::openPinch, 900);
////        }
////    }
//
//
////    public void readyBucketPos(boolean input, boolean aimHigh) {
////        if (input) {
////            rotateToVerticalPos();
////            if (aimHigh) {
////                pivotToHighBasketPos();
////                moveSlideToHighBucketPos();
////            } else {
////                pivotToLowBasketPos();
////                moveSlideToLowBucketPos();
////            }
////        }
////    }
////    /**
////     * This method will score a bucket (pivot arm a little, open pinch, pivot arm up)
////     * @param input - boolean to determine if the robot should score a bucket
////     */
////    public void scoreBucketSimple(boolean input) {
////        if (input) {
////            pivotByDelta(-50);
////            schedule(this::openPinch, 100);
////            schedule(()->pivotByDelta(100), 300);
////        }
////    }
//
//
//
//
//

}
