package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlide {

    //Declare Motor Variable
    private DcMotor linSlideMtr = null;
    private float minPosition;
    private float maxPosition;

    /**
     * Constructor
     *
     * @param slide DcMotor of slider
     * @param minConstraint min encoder position of motor between 0 and 360 (float)
     * @param maxConstraint max encoder position of motor between 0 and 360 (float)
     */
    public LinearSlide(DcMotor slide, float minConstraint, float maxConstraint){
        this.linSlideMtr = slide;
        
        this.minPosition = minConstraint;
        this.maxPosition = maxConstraint;

        this.linSlideMtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.linSlideMtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    /**
     * MoveSlideUnrestricted()
     *
     * @param speed speed to move slide (-1.0f, 1.0f)
     */
    public void MoveSlideUnrestricted(float speed){
        this.linSlideMtr.setPower(speed);
    }


//
//    /**
//     * @return current encoder position of motor
//     */
//    public int getCurrentPosition(){
//        return linSlideMtr.getCurrentPosition();
//    }
//
//    /**
//     * moveToPosition()
//     *
//     * @param position encoder position to go to
//     * @param speed motor speed (-1 to 1)
//     * @return current encoder position
//     */
//    public float moveToPosition(int position, double speed){
//        if(this.minPosition < position && position < this.maxPosition){
//            linSlideMtr.setTargetPosition(position);
//            linSlideMtr.setPower(speed);
//
//        }
//
//        return getCurrentPosition();
//    }
//
//    /**
//     * moveRelative
//     *
//     * @param offset offset to move by
//     * @param speed
//     * @return
//     */
//    public float moveRelative(int offset, double speed){
//
//        return moveToPosition((getCurrentPosition() + offset), speed);
//    }

}
