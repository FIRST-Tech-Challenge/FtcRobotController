package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.utils.LocalizerInterface;

/**
 * This will be the localizer that will combine
 * any and all localization data into a single location
 */
public class PrimaryLocalizer implements LocalizerInterface, Localizer {

    //Should be equal or be close to 1;
    private double totalWeight = 0;
    private LocalizerInterface[] localizers;

    private double lastPosX, lastPosY;
    private Rotation2d lastHeading;
    private boolean initialized;
    private long lastCalled;

    /**
     * Constructor to create a PriamryLocalizer Class
     * @param localizers [LocalizerInterface[]] An array of LocalizerInterface(s) to analyze
     */
    public PrimaryLocalizer(LocalizerInterface[] localizers){
        this.localizers = localizers;
        for(LocalizerInterface sensors: localizers){
            totalWeight += sensors.getWeight();
        }
    }


    /**
     * @return [double] Returns weight of all combined localizers.
     */
    @Override //Unncessary for this particular localizer, but can be used to check total weight
    public double getWeight() {return totalWeight;}


    /**Currently, this simply takes in the position (in Pose2d form) from all localizers
     * Each localizer is then multipled with it's associated weight.
     * @return [Pose2d] Returns a Pose2d that reflects the position of the robot.
     */
    @Override
    public Pose2d getPosition() {
        double xPos = 0, yPos = 0, heading = 0;
        for(LocalizerInterface sensors: localizers){
            Pose2d pos = sensors.getPosition();
            xPos += pos.position.x * (sensors.getWeight()/totalWeight);
            yPos += pos.position.y * (sensors.getWeight()/totalWeight);
            //For some reason, not applying an average calculation gives the correct output?
            heading += pos.heading.toDouble() /* *(sensors.getWeight()/totalWeight)*/;
        }
        return new Pose2d( xPos , yPos , heading );
    }


    /**
     * Returns position and heading data
     * @return [String] Formatted string containing position and heading data.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        Pose2d pos = getPosition();
        return String.format("X: %f\nY: %f\nHeading: %f",
                pos.position.x,
                pos.position.y,
                Math.toDegrees(pos.heading.toDouble()));
    }


    public LocalizerInterface[] getLocalizers(){
        return localizers;
    }

    @Override
    public Twist2dDual<Time> update() {
        Pose2d pos = getPosition();
        Rotation2d heading = Rotation2d.exp(pos.heading.toDouble());
        if (!initialized){
            lastPosX = pos.position.x;
            lastPosY = pos.position.y;
            lastHeading = heading;
            lastCalled = System.currentTimeMillis();
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        double xDelta = pos.position.x - lastPosX;
        double yDelta = pos.position.y - lastPosY;
        double headingDelta = heading.minus(lastHeading);
        long timeNow = System.currentTimeMillis();
        double timeDeltaSecods = (timeNow - lastCalled)/1000.0;
        double xVel = xDelta / timeDeltaSecods;
        double yVel = yDelta / timeDeltaSecods;
        double headingVel = headingDelta / timeDeltaSecods;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                xDelta,
                                xVel,
                        }),
                        new DualNum<Time>(new double[] {
                                yDelta,
                                yVel,
                        })
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );


        lastPosX = pos.position.x;
        lastPosY = pos.position.y;
        lastHeading = heading;
        lastCalled = timeNow;

        return twist;
    }
}


