package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
                pivotToSpecimenPos();

                timer.reset();
                while (timer.milliseconds() < 1000){}
                //rotateToPos(0);


                timer.reset(); // Reset the timer again
                while (timer.milliseconds() < 5000) {
                    // Wait another second
                }

                moveSlideToSpecimenPos();

            }

            if (getPivotPosition() >= //specimenSlidePosition -100 && getPivotPosition() <= specimentSlidePosition + 100) {
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
    public void scoreBucket(boolean input){
        if (input) {
            if (getPivotPosition() <= minimumPivotPos + 100 && getPivotPosition() >= minimumPivotPos -50 ) {
                timer.reset(); // Reset the timer
                while (timer.milliseconds() < 1000) {
                    // Allow other tasks to run, like telemetry updates
                }
                pivotToBucketPos();


                timer.reset(); // Reset the timer again
                while (timer.milliseconds() < 5000) {
                    // Wait another second
                }

                moveSlideToBucketPos();

            }

            if (getPivotPosition() >= bucketSlidePosition -100 && getPivotPosition() <= bucketSlidePosition + 100) {


                timer.reset();
                while (timer.milliseconds() < 500) {
                    // Wait for 2 seconds before pinching
                }
                openPinch();

            }
        }
    }
    
        




}
