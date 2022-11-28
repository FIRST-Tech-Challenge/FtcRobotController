package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;
import java.util.Map;

/** Controls motors and servos that are not involved in moving the robot around the field.
 */
public class MechanismDriving {

    private static int desiredSlidePosition;

    // TODO: empirically measure values of slides positions
    // TODO: empirically measure number of encoder counts for lowering horseshoe
    public static final int LOWERING_AMOUNT = 100;
    public static final Map<Robot.SlidesState, Integer> slidePositions = new HashMap<Robot.SlidesState, Integer>() {{
       put(Robot.SlidesState.RETRACTED, 0);
       put(Robot.SlidesState.LOW_LOWERED, slidePositions.get(Robot.SlidesState.LOW) - LOWERING_AMOUNT);
       put(Robot.SlidesState.LOW, 700);
       put(Robot.SlidesState.MEDIUM_LOWERED, slidePositions.get(Robot.SlidesState.MEDIUM) - LOWERING_AMOUNT);
       put(Robot.SlidesState.MEDIUM, 1850);
       put(Robot.SlidesState.HIGH_LOWERED, slidePositions.get(Robot.SlidesState.HIGH) - LOWERING_AMOUNT);
       put(Robot.SlidesState.HIGH, 3500);
    }};
    //public static final int RETRACTED_POS = 0, LOW_POS = 700, MEDIUM_POS = 1850, HIGH_POS = 3500;
    public static final double HORSESHOE_FRONT_POS = 0, HORSESHOE_REAR_POS = 1.0; //These are not final values
    public static final double COMPLIANT_WHEELS_SPEED = 1.0; //speed of compliant wheels
    // How long it takes for the horseshoe wheels to be guaranteed to have pushed the cone into the horseshoe.
    public static final long HORSESHOE_TIME = 500;
    public static final int EPSILON = 50;  // slide encoder position tolerance;

    double slideRampDownDist=1000, maxSpeedCoefficient=0.8, reducedSpeedCoefficient=0.7;


    public static final int slidesAdjustmentSpeed = 2;

    MechanismDriving() {}

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
        switch (robot.desiredCompliantWheelsState) {
            case OFF:
                robot.compliantWheels.setPower(0);
                break;
            case ON:
                robot.compliantWheels.setPower(COMPLIANT_WHEELS_SPEED);
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

        if(Robot.desiredSlidesState != Robot.SlidesState.UNREADY){
            setSlidePosition(robot, slidePositions.get(Robot.desiredSlidesState));

            double mainSpeed,reducedSpeed;//"ramp" the motor speeds down based on how far away from the destination the motors are
            mainSpeed= maxSpeedCoefficient *Range.clip(Math.abs(desiredSlidePosition - robot.slidesRight.getCurrentPosition())/slideRampDownDist, 0.1, 1);
            reducedSpeed= reducedSpeedCoefficient *Range.clip(Math.abs(desiredSlidePosition - robot.slidesRight.getCurrentPosition())/slideRampDownDist, 0.1, 1);
            mainSpeed=Range.clip(mainSpeed,0.4, 1);//limit the max speed to 1 and the min speed to 0.05
            reducedSpeed=Range.clip(reducedSpeed,0.3,1);

            // If the current position is less than desired position then move it up
            if (desiredSlidePosition - robot.slidesRight.getCurrentPosition() > EPSILON) {
                // Ensures that one motor does not go beyond the other too much
                if (robot.slidesLeft.getCurrentPosition() == robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(mainSpeed);
                    robot.slidesRight.setPower(mainSpeed);
                }
                else if(robot.slidesLeft.getCurrentPosition() > robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(reducedSpeed);
                    robot.slidesRight.setPower(mainSpeed);
                }
                else if(robot.slidesLeft.getCurrentPosition() < robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(mainSpeed);
                    robot.slidesRight.setPower(reducedSpeed);
                }
            }

            // If the current position is above the current position, move these downwards
            if (robot.slidesRight.getCurrentPosition() - desiredSlidePosition > EPSILON) {
                // Ensures that one motor does not go beyond the other too much
                if (robot.slidesLeft.getCurrentPosition() == robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(-mainSpeed); // Go in the opposite direction
                    robot.slidesRight.setPower(-mainSpeed);
                }
                else if (robot.slidesLeft.getCurrentPosition() < robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(-reducedSpeed);
                    robot.slidesRight.setPower(-mainSpeed);
                }
                else if (robot.slidesLeft.getCurrentPosition() > robot.slidesRight.getCurrentPosition()) {
                    robot.slidesLeft.setPower(-mainSpeed);
                    robot.slidesRight.setPower(-reducedSpeed);
                }
            }
//            robot.telemetry.addData("slides: target: ",desiredSlidePosition+" current pos right: "+robot.slidesRight.getCurrentPosition()+ " current pos left: "+robot.slidesLeft.getCurrentPosition());
            // Stop motors when we have reached the desired position
            if (Math.abs(robot.slidesRight.getCurrentPosition() - desiredSlidePosition) < EPSILON) {
                robot.slidesLeft.setPower(0);
                robot.slidesRight.setPower(0);
                return true;
            }
            else {
                return false;
            }
        }
        return false;
    }
}
