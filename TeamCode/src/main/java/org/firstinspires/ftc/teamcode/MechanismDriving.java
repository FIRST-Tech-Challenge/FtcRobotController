package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {

    private static int desiredSlidePosition;
    public boolean testing=false;

    public static final Map<Robot.SlidesState, Integer> slidePositions = new HashMap<Robot.SlidesState, Integer>() {{
       put(Robot.SlidesState.RETRACTED, 0);
       put(Robot.SlidesState.LOW, 4770);
       put(Robot.SlidesState.MEDIUM, 8020);
       put(Robot.SlidesState.HIGH, 13270);
    }};
    public static final double CLAW_CLOSED_POS = 0, CLAW_OPEN_POS = 0.8; //These are not final values
    // How long it takes for the claw servo to be guaranteed to have moved to its new position.
    public static final long CLAW_SERVO_TIME = 500;
    //SPEED INFO: Scale from 0-1 in speed.
    public static final double CLAW_ROTATOR_FRONT_POS = 0, CLAW_ROTATOR_REAR_POS = 1.0, CLAW_ROTATOR_SIDE_POS = 0.45;
    // How long it takes for the horseshoe wheels to be guaranteed to have pushed the cone into the horseshoe.
    public static final long HORSESHOE_TIME = 500;
    public static final int EPSILON = 50;  // slide encoder position tolerance;

    public static final double SLIDE_RAMP_DIST = 400;
    public static final double SLIDES_MAX_SPEED = 1;
    public static final double SLIDE_MIN_SPEED = 0.4;


    public static final int slidesAdjustmentSpeed = 2;

    public MechanismDriving() {
//        slidePositions.put(Robot.SlidesState.LOW_LOWERED, slidePositions.get(Robot.SlidesState.LOW) - LOWERING_AMOUNT);
//        slidePositions.put(Robot.SlidesState.MEDIUM_LOWERED, slidePositions.get(Robot.SlidesState.MEDIUM) - LOWERING_AMOUNT);
//        slidePositions.put(Robot.SlidesState.HIGH_LOWERED, slidePositions.get(Robot.SlidesState.HIGH) - LOWERING_AMOUNT);
    }

    /** Sets the claw position to the robot's desired state.
     */
    public void updateClaw(Robot robot) {
        switch (robot.desiredClawState) {
            case CLOSED:
                robot.claw.setPosition(CLAW_CLOSED_POS); //closed
                break;
            case OPEN:
                robot.claw.setPosition(CLAW_OPEN_POS); //open
                break;
        }
    }

    /** Sets the claw position to the robot's desired state.
     */
    public void updateClawRotator(Robot robot) {
        switch (robot.desiredClawRotatorState) {
            case FRONT:
                robot.clawRotator.setPosition(CLAW_ROTATOR_FRONT_POS); //facing the front of the robot
                break;
            case SIDE:
                robot.clawRotator.setPosition(CLAW_ROTATOR_SIDE_POS);
                break;
            case REAR:
                robot.clawRotator.setPosition(CLAW_ROTATOR_REAR_POS); //facing the rear of the robot
                break;
        }
    }

    /** Sets the preferred position of the slides
     *
     * @param position - The encoder count that the motors for the slide should get to
     */
    public void setSlidePosition(Robot robot, int position) {
        desiredSlidePosition = position;
    }

    /** Returns the target encoder count given the robot's desired slides state.
     */
    public int getTargetSlidesEncoderCount(Robot robot) {
        int encoderCount;
        switch (Robot.desiredSlidesState) {
            case MOVE_UP:
                encoderCount = robot.slidesMotor.getCurrentPosition() + EPSILON + 1;
            case MOVE_DOWN:
                encoderCount = robot.slidesMotor.getCurrentPosition() + EPSILON + 1;
            default:
                try {
                    encoderCount = slidePositions.get(Robot.desiredSlidesState);
                } catch (Exception e) {
                    robot.telemetry.addData("desired slide state", Robot.desiredSlidesState.toString());
                    try {
                        robot.elapsedTime.wait(10000);
                    } catch (InterruptedException ex) {
                        throw new RuntimeException(ex);
                    }
                    throw e;
                }
        }
        // TODO: this negation was added during the competition but it may not be necessary.
        return -encoderCount;
    }

    /** Sets slide motor powers to move in direction of desired position, if necessary.
     *
     * @return whether the slides are in the desired position.
     */
    public boolean updateSlides(Robot robot, double slidesPower) {

       if (Robot.desiredSlidesState != Robot.SlidesState.UNREADY) {
           if(!testing)
               setSlidePosition(robot, getTargetSlidesEncoderCount(robot));

           // Speed is proportional to the fraction of the ramp distance that we have left.
           double slidesSpeed = slidesPower * Range.clip(Math.abs(desiredSlidePosition - robot.slidesMotor.getCurrentPosition())/ SLIDE_RAMP_DIST, SLIDE_MIN_SPEED, 1);

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
