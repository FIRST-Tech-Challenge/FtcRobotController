package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutomationBot extends LimelightBot{


    public AutomationBot(LinearOpMode opMode) {
        super(opMode);
    }


    public void init(HardwareMap hardwareMap){
        super.init(hardwareMap);


    }

    protected void onTick() {
        super.onTick();

    }
    public void scoreSpecimen(boolean input) {
        if (input) {
            if (getPivotPosition() <= 150 && getPivotPosition() >= 50 ) {
                timer.reset(); // Reset the timer
                while (timer.milliseconds() < 1000) {
                    // Allow other tasks to run, like telemetry updates
                }
                pivotToSpecimenHighPos();

                timer.reset();
                while (timer.milliseconds() < 1000){}
                //rotateToPos(0);


                timer.reset(); // Reset the timer again
                while (timer.milliseconds() < 5000) {
                    // Wait another second
                }

                moveSlideToHighSpecimenPos();

            }

            if (getPivotPosition() >= specimenHighSlidePos -100 && getPivotPosition() <= specimenHighSlidePos + 100) {
                timer.reset();
                while (timer.milliseconds() < 1000) {
                    // Wait before moving the slide
                }

                //moveSlideToSpecimenScorePos();

                timer.reset();
                while (timer.milliseconds() < 500) {
                    // Wait for 2 seconds before pinching
                }

                openPinch();
                slideTarget = 100;
                pivotTarget = 100;
            }
        }
    }

    /**
     * This method will prepare the robot to score a specimen (raise pivot arm, extend slide to a certain position)
     * @param input - boolean to determine if the robot should prepare to score a specimen
     * @param aimHigh - boolean to determine if the robot should aim high or low chamber
     */
    public void readySpecimenPos(boolean input, boolean aimHigh) {
        if (input) {
            rotateToVerticalPos();
            if (aimHigh) {
                pivotToSpecimenHighPos();
                moveSlideToHighSpecimenPos();
            } else {
                pivotToSpecimenLowPos();
                moveSlideToLowSpecimenPos();
            }
        }
    }

    /**
     * This method will score a specimen (slide down a little, open pinch)
     * @param input
     */
    public void scoreSpecimenSimple(boolean input) {
        if (input) {
            moveSlideByDelta(-100);
            schedule(this::openPinch, 300);
        }
    }

    /**
     * This method will prepare the robot to score a basket (raise pivot arm, extend slide to a certain position)
     * @param input - boolean to determine if the robot should prepare to score a basket
     * @param aimHigh - boolean to determine if the robot should aim high or low basket
     */
    public void readyBucketPos(boolean input, boolean aimHigh) {
        if (input) {
            rotateToVerticalPos();
            if (aimHigh) {
                pivotToHighBasketPos();
                moveSlideToHighBucketPos();
            } else {
                pivotToLowBasketPos();
                moveSlideToLowBucketPos();
            }
        }
    }
    /**
     * This method will score a bucket (pivot arm a little, open pinch, pivot arm up)
     * @param input - boolean to determine if the robot should score a bucket
     */
    public void scoreBucketSimple(boolean input) {
        if (input) {
            pivotByDelta(-50);
            schedule(this::openPinch, 100);
            schedule(()->pivotByDelta(100), 300);
        }
    }

    public void scoreBucket(boolean input){
        if (input) {
            if (getPivotPosition() <= minumimPivotPos + 100 && getPivotPosition() >= minumimPivotPos -50 ) {
                timer.reset(); // Reset the timer
                while (timer.milliseconds() < 1000) {
                    // Allow other tasks to run, like telemetry updates
                }
                pivotToHighBasketPos();


                timer.reset(); // Reset the timer again
                while (timer.milliseconds() < 5000) {
                    // Wait another second
                }

                moveSlideToHighBucketPos();

            }

            if (getPivotPosition() >= highBucketSlidePos -100 && getPivotPosition() <= highBucketSlidePos + 100) {


                timer.reset();
                while (timer.milliseconds() < 500) {
                    // Wait for 2 seconds before pinching
                }
                openPinch();

            }
        }
    }
    
        




}
