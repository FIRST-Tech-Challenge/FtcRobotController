package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.utils.LocalizerInterface;

/**
 * Modified version of ThreeDeadWheelLocalizer
 * TODO: Make sure to adjust the Params in ThreeDeadWheelLocalizer.java
 */
public class ThreeEncoderLocalizer extends ThreeDeadWheelLocalizer implements LocalizerInterface {


    public double weight = 0.33;
    private Pose2d position;

    String debugValue;
    public ThreeEncoderLocalizer(HardwareMap hw){
        this(hw, new String[] {"FLM","BRM","BLM"});
    }

    public ThreeEncoderLocalizer(HardwareMap hw, String[] names){
        super(hw, 0.0029, names);
        position = new Pose2d(0,0,0);
    }
    public ThreeEncoderLocalizer(HardwareMap hardwareMap, double inPerTick) {
        super(hardwareMap, inPerTick);
        position = new Pose2d(0,0,0);
    }

    @Override
    public double getWeight() {
        return weight;
    }

    /**
     *
     * @return Pose2d object in (inches, inches, radians)
     */
    @Override
    public Pose2d getPosition() {

        Twist2dDual<Time> twist = super.update();
        debugValue = "" + twist.value().line.x + " , " + twist.value().line.y + " , " + twist.value().angle;
        if (twist != null){
            position = position.plus(twist.value());
        }
        Pose2d result = new Pose2d(position.position.x, position.position.y, Math.toRadians(position.heading.toDouble()));
        return result;
    }

    public String toString(){
        return String.format("X: %f\nY: %f\nHeading: %f",
                position.position.x,
                position.position.y,
                Math.toDegrees(position.heading.toDouble()));
    }

    public String DebugLog(){
        return debugValue;
    }
}
