package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.utils.LocalizerInterface;

/**
 * This will be the localizer that will combine
 * any and all localization data into a single location
 */
public class PrimaryLocalizer implements LocalizerInterface {

    //Should be equal or be close to 1;
    private double totalWeight = 0;
    private LocalizerInterface[] localizers;

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
}


