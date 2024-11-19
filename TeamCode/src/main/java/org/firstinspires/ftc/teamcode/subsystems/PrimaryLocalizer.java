package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.Localizer;
import org.firstinspires.ftc.teamcode.utils.LimeLightWrapper;
import org.firstinspires.ftc.teamcode.utils.LocalizerInterface;

/**
 * This will be the localizer that will combine
 * any and all localization data into a single location
 */
public class PrimaryLocalizer implements LocalizerInterface, Localizer {

    //Should be equal or be close to 1;
    private double totalWeight = 0;
    private LocalizerInterface[] localizers;

    private LimeLightWrapper.Color currentColor = LimeLightWrapper.Color.BLUE_SIDE;
    private double lastPosX, lastPosY;
    private Rotation2d lastHeading;
    private boolean initialized;
    private long lastCalled;

    private Pose2d initialPosition;

    /**
     * Constructor to create a PriamryLocalizer Class
     * @param localizers [LocalizerInterface[]] An array of LocalizerInterface(s) to analyze
     */
    public PrimaryLocalizer(LocalizerInterface[] localizers){
        this.localizers = localizers;
        for(LocalizerInterface sensors: localizers){
            totalWeight += sensors.getWeight();
        }
//        setInitialPosition(new Pose2d(0,0,0));
    }
    public PrimaryLocalizer(LocalizerInterface[] localizers, Pose2d pose){
        this(localizers);
        lastHeading = pose.heading;
        setInitialPosition(pose);
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
        double xPos = 0, yPos = 0, heading = 0, weight = 0;
        for(LocalizerInterface sensors: localizers){
            if(sensors.isValid()) {
                Pose2d pos = sensors.getPosition();//addPose2d(initialPosition, sensors.getPosition());
                if (initialized && sensors instanceof LimeLightWrapper){
                    LimeLightWrapper limeLight = (LimeLightWrapper) (sensors);
                    pos = limeLight.getPosition(lastHeading.toDouble());
                }
                weight += sensors.getWeight();
                xPos += pos.position.x * (sensors.getWeight());
                yPos += pos.position.y * (sensors.getWeight());
                //For some reason, not applying an average calculation gives the correct output?
                heading += pos.heading.toDouble() * (sensors.getWeight());
            }
        }
        //Average based on weight
        xPos /= weight;
        yPos /= weight;
        heading /= weight;
        return new Pose2d( xPos , yPos , heading );
    }

    @Override
    public boolean isValid() {
        return true;
    }


    /**
     * Returns position and heading data
     * @return [String] Formatted string containing position and heading data.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public String toString(){
        Pose2d pos = getPosition();
        return String.format("Average:\nX: %f\nY: %f\nHeading: %f\n",
                pos.position.x,
                pos.position.y,
                Math.toDegrees(pos.heading.toDouble()));
    }

    @SuppressLint("DefaultLocale")
    public String allLocalizersToString(){
        StringBuilder str = new StringBuilder(toString());
        int idx = 1;
        for(LocalizerInterface sensors: localizers){
            if(sensors.isValid()) {
                Pose2d pos = sensors.getPosition();//addPose2d(initialPosition, sensors.getPosition());
                if (sensors instanceof LimeLightWrapper){
                    LimeLightWrapper limeLight = (LimeLightWrapper) (sensors);
                    pos = limeLight.getPosition(lastHeading.toDouble());
                }
                str.append(String.format("Sensor %d:\nX: %f\nY: %f\nHeading: %f\n",
                        idx,
                        pos.position.x,
                        pos.position.y,
                        Math.toDegrees(pos.heading.toDouble())));
                idx++;
            }
        }
        return str.toString();
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
            initialized = true;
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        //Get the change in position in Field Centric X and Y
        double xDelta = pos.position.x - lastPosX ;
        double yDelta = pos.position.y - lastPosY;

        //Field centric --> Robot Centric Coordinates
        double traveledDist = Math.sqrt(Math.pow(xDelta,2) + Math.pow(yDelta,2));
        double localHeading = Math.atan2(yDelta, xDelta) - heading.toDouble()   ;
        xDelta =  traveledDist * Math.cos(localHeading);
        yDelta =  traveledDist * Math.sin(localHeading);;

        double headingDelta = heading.minus(lastHeading);

        //Calculate Velocities
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
                        }).times(1),
                        new DualNum<Time>(new double[] {
                                yDelta,
                                yVel,
                        }).times(1)
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


    public void setColor(LimeLightWrapper.Color color){
        this.currentColor = color;
        for(LocalizerInterface localizer: localizers){
            localizer.setColor(color);
        }
    }

    @Override
    public void setInitialPosition(Pose2d pose) {
        for(LocalizerInterface localizer: localizers){
            localizer.setInitialPosition(pose);
        }
    }

    private Pose2d addPose2d(Pose2d pos1, Pose2d pos2){

        double x = pos1.position.x + pos2.position.x;
        double y = pos1.position.y + pos2.position.y;
        double h = pos1.heading.toDouble() + pos2.heading.toDouble();

        return new Pose2d(x,y,h);
    }
}


