package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {

    private static int desiredSlidePosition;
    public boolean testing=false;

    // TODO: empirically measure values of slides positions
    // TODO: empirically measure number of encoder counts for lowering horseshoe
    public static final int LOWERING_AMOUNT = 100;
    public static final Map<Robot.SlidesState, Integer> slidePositions = new HashMap<Robot.SlidesState, Integer>() {{
       put(Robot.SlidesState.RETRACTED, 0);
       put(Robot.SlidesState.LOW, 1670);
       put(Robot.SlidesState.MEDIUM, 2800);
       put(Robot.SlidesState.HIGH, 3870);//may need to be higher
    }};
    //SPEED INFO: Scale from 0-1 in speed.
    public static final double HORSESHOE_FRONT_POS = 0, HORSESHOE_REAR_POS = 1.0; //These are not final values
    public static final double COMPLIANT_WHEELS_SPEED = 1.0; //speed of compliant wheels
    // How long it takes for the horseshoe wheels to be guaranteed to have pushed the cone into the horseshoe.
    public static final long HORSESHOE_TIME = 500;
    public static final long COMPLIANT_WHEELS_TIME = 2000;
    public static final int EPSILON = 50;  // slide encoder position tolerance;

    public static final double SLIDE_RAMP_DIST = 333;
    public static final double SLIDES_MAX_SPEED = 1;


    public static final int slidesAdjustmentSpeed = 2;

    public MechanismDriving() {
        slidePositions.put(Robot.SlidesState.LOW_LOWERED, slidePositions.get(Robot.SlidesState.LOW) - LOWERING_AMOUNT);
        slidePositions.put(Robot.SlidesState.MEDIUM_LOWERED, slidePositions.get(Robot.SlidesState.MEDIUM) - LOWERING_AMOUNT);
        slidePositions.put(Robot.SlidesState.HIGH_LOWERED, slidePositions.get(Robot.SlidesState.HIGH) - LOWERING_AMOUNT);
    }

    /** Sets the horseshoe position to the robot's desired state.
     */
    public void updateHorseshoe(Robot robot) {
        switch (robot.desiredHorseshoeState) {
            case FRONT:
                robot.horseshoe.setPosition(HORSESHOE_FRONT_POS); //facing the front of the robot
                robot.horseshoeIndicator.setPosition(0);
                break;
            case REAR:
                robot.horseshoe.setPosition(HORSESHOE_REAR_POS); //facing the rear of the robot
                robot.horseshoeIndicator.setPosition(1);
                break;
        }
    }

    /** Starts and stops the compliant wheels
     *
     * @param robot The robot
     */
    public void updateCompliantWheels(Robot robot) {
//        switch (robot.desiredCompliantWheelsState) {
//            case OFF:
//                robot.compliantWheelsMotor.setPower(0);
//                break;
//            case ON:
//                robot.compliantWheelsMotor.setPower(COMPLIANT_WHEELS_SPEED);
//                break;
//        }
    }

    /** Sets the preferred position of the slides
     *
     * @param position - The encoder count that the motors for the slide should get to
     */
    public void setSlidePosition(Robot robot, int position) {
        desiredSlidePosition = position;
    }

    /** Sets desired states of slide motor powers using joystick.
     *
     * TODO: implement this
     */
    public void adjustDesiredSlideHeight(AnalogValues analogValues, Robot robot) {
//        if (analogValues.gamepad2LeftStickY < -RobotManager.JOYSTICK_DEAD_ZONE_SIZE) {
//            Robot.desiredSlidesState
//        }
    }

    /** Sets slide motor powers to move in direction of desired position, if necessary.
     *
     * @return whether the slides are in the desired position.
     */
    public boolean updateSlides(Robot robot) {

       if (Robot.desiredSlidesState != Robot.SlidesState.UNREADY) {
           if(!testing)
               setSlidePosition(robot, slidePositions.get(Robot.desiredSlidesState));

           // Speed is proportional to the fraction of the ramp distance that we have left.
           double slidesSpeed = SLIDES_MAX_SPEED * Range.clip(Math.abs(desiredSlidePosition - robot.slidesMotor.getCurrentPosition())/ SLIDE_RAMP_DIST, 0.1, 1);
           slidesSpeed = Range.clip(slidesSpeed, 0.4, 1);

           // If the current position is less than desired position then move it up
           if (desiredSlidePosition - robot.slidesMotor.getCurrentPosition() > EPSILON) {
               robot.slidesMotor.setPower(slidesSpeed);
           }

           // If the current position is above the desired position, move these downwards
           if (robot.slidesMotor.getCurrentPosition() - desiredSlidePosition > EPSILON) {
               robot.slidesMotor.setPower(-slidesSpeed);
           }

           robot.telemetry.addData("slides: target: ",desiredSlidePosition+" current pos: "+robot.slidesMotor.getCurrentPosition());

           // Stop motors when we have reached the desired position
           if (Math.abs(robot.slidesMotor.getCurrentPosition() - desiredSlidePosition) < EPSILON) {
               robot.slidesMotor.setPower(0);
               return true;
           }
           return false;
       }
       return false;
    }
}
