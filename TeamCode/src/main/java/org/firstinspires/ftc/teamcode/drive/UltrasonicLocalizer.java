package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.mechanism.MB1242;

import java.util.ArrayList;
import java.util.List;

public class UltrasonicLocalizer implements Localizer {
    ElapsedTime pingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    MB1242 sensor1;
    MB1242 sensor2;
    MB1242 sensor3;
    MB1242 sensor4;

    MecanumDrive drive;

    /**
     *
     * @param sensors An array of 4 MB1242 ultrasonic sensors, in order of the robot's front, right, back, left
     * @param drive The MecanumDrive object to use for localization
     */
    UltrasonicLocalizer(MB1242[] sensors, MecanumDrive drive) {
        sensor1 = sensors[0];
        sensor2 = sensors[1];
        sensor3 = sensors[2];
        sensor4 = sensors[3];
        this.drive = drive;
    }

    double x = 0;
    double y = 0;
    double theta = 0;
    Pose2d previousPose = new Pose2d(0, 0, 0);
    Pose2d poseEstimate = new Pose2d(0, 0, 0);
    List<Double> lastWheelPositions = new ArrayList<>();
    Pose2d poseVelocity;


    public void init(HardwareMap hardwareMap) {
        pingTimer.reset();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // A guy on discord said this is about right
    int MILLIS_PER_CYCLE = 34;

    void pingSensors() {
        sensor1.ping();
        sensor2.ping();
        sensor3.ping();
        sensor4.ping();
    }

    final double ROBOT_WIDTH = 11.95;
    final double ROBOT_LENGTH = 13.8;

    // This gets the distances from all 4 sensors and adds the robot width and length
    // to the distances where necessary
    double[] getSensorDistances() {
        double[] distances = new double[4];
        distances[0] = sensor1.getDistance(DistanceUnit.INCH);
        distances[1] = sensor2.getDistance(DistanceUnit.INCH);
        distances[2] = sensor3.getDistance(DistanceUnit.INCH);
        distances[3] = sensor4.getDistance(DistanceUnit.INCH);
        distances[0] += ROBOT_LENGTH / 2;
        distances[1] += ROBOT_WIDTH / 2;
        distances[2] += ROBOT_LENGTH / 2;
        distances[3] += ROBOT_WIDTH / 2;
        return distances;
    }


    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        previousPose = pose2d;
        poseEstimate = pose2d;
        lastWheelPositions = new ArrayList<>();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    @Override
    public void update() {
        // Calculate the position of the robot based on the ultrasonic sensors and the IMU values
        // Remember that the field is 12 x 12 feet, and the robot is 11.95 x 13.8 inches
        // The minimum distance detectable by the ultrasonic sensors is 7.874 inches
        // The maximum distance is irrelevant, as the robot will never be that far away from a wall
        theta = drive.getExternalHeading();
        if (pingTimer.time() > MILLIS_PER_CYCLE) {
            pingTimer.reset();
            pingSensors();
            double[] distances = getSensorDistances();
            ArrayList<Pose2d> averagablePoses = new ArrayList<>();
            // All the different pairs of distances we can use to calculate the position of the robot
            double[] set1 = {distances[0], distances[1]};
            double[] set2 = {distances[1], distances[2]};
            double[] set3 = {distances[2], distances[3]};
            double[] set4 = {distances[3], distances[0]};
            double[] set5 = {distances[0], distances[2]};
            double[] set6 = {distances[1], distances[3]};
            // Make a list of them
            double[][] sets = {set1, set2, set3, set4, set5, set6};
            // For each set, calculate the position of the robot that you could get from those
            for (double[] set : sets) {
                /* If either of the distances is less than 7.874 or greater than 144 inches, then
                we can't calculate the position of the robot using these measurements,
                so we skip this set */
                if (set[0] < 7.874 || set[0] > 144 || set[1] < 7.874 || set[1] > 144) {
                    continue;
                }

                // Let me know if you need help understanding this part,
                // It's pretty basic trig though

                double x1 = set[0] * Math.cos(theta);
                double y1 = set[0] * Math.sin(theta);
                double x2 = set[1] * Math.cos(theta);
                double y2 = set[1] * Math.sin(theta);

                double correctedX1;
                double correctedY1;
                double correctedX2;
                double correctedY2;

                // If any of them are negative, then subtract them from 144 (the size of the field)
                // to get the real position
                // We still need the originals to figure which reading to use.
                if (x1 < 0) {
                    correctedX1 = 144 + x1;
                } else {
                    correctedX1 = x1;
                }
                if (y1 < 0) {
                    correctedY1 = 144 + y1;
                } else {
                    correctedY1 = y1;
                }
                if (x2 < 0) {
                    correctedX2 = 144 + x2;
                } else {
                    correctedX2 = x2;
                }
                if (y2 < 0) {
                    correctedY2 = 144 + y2;
                } else {
                    correctedY2 = y2;
                }

                // We assume that the larger distance is the one that is more likely to be the wall
                // Rather than an object in the way
                double realX = Math.abs(x1) > Math.abs(x2) ? correctedX1 : correctedX2;
                double realY = Math.abs(y1) > Math.abs(y2) ? correctedY1 : correctedY2;

                // If the x or y values are very similar, the two sensors are probably facing the same wall
                // And the pose they calculate is probably wrong, so we don't use it
                if (Math.abs(x1 - x2) < 5) {
                    continue;
                }
                if (Math.abs(y1 - y2) < 5) {
                    continue;
                }


                Pose2d newPose = new Pose2d(realX, realY, theta);
                double poseDifference = newPose.vec().distTo(previousPose.vec());
                // If the pose difference is too big, we don't use it, since again, it's probably wrong
                // This is the main filter that makes sure no measurements too insane get used
                // It is a bit jank though, a Kalman filter would be a better way to do this
                if (poseDifference < 2) {
                    averagablePoses.add(newPose);
                }
            }
            // At the end, we average all the realistic poses we found
            if (averagablePoses.size() > 0) {
                double xAverage = 0;
                double yAverage = 0;
                for (Pose2d pose : averagablePoses) {
                    xAverage += pose.getX();
                    yAverage += pose.getY();
                }
                xAverage /= averagablePoses.size();
                yAverage /= averagablePoses.size();
                poseEstimate = new Pose2d(xAverage, yAverage, theta);
            } else {
                // If we didn't find any poses, we use the encoder values instead
                poseEstimate = calculatePoseEncoders();
                poseVelocity = calculatePoseDeltaEncoders();
            }
        } else {
            // If we haven't waited long enough, we use the encoder values instead
            poseEstimate = calculatePoseEncoders();
            poseVelocity = calculatePoseDeltaEncoders();
        }
        previousPose = poseEstimate;
    }
    /**
     * @see com.acmerobotics.roadrunner.drive.MecanumDrive.MecanumLocalizer
     * Basically just that math added to this class because some of it is private
     */
    public Pose2d calculatePoseEncoders() {
        List<Double> wheelPositions = drive.getWheelPositions();
        List<Double> wheelDeltas = new ArrayList<>();
        if (lastWheelPositions.size() != 0) {
            for (int i = 0; i < lastWheelPositions.size(); i++) {
                double difference = wheelPositions.get(i) - lastWheelPositions.get(i);
                wheelDeltas.add(difference);
            }
        }
        Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                wheelDeltas,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.TRACK_WIDTH
        );
        double finalHeadingDelta = Angle.normDelta(robotPoseDelta.getHeading() - previousPose.getHeading());
        lastWheelPositions = wheelPositions;
        return Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
    }
    public Pose2d calculatePoseDeltaEncoders() {
        Pose2d poseVelocity;
        List<Double> wheelVelocities = drive.getWheelVelocities();
        if (wheelVelocities != null) {
            poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                    wheelVelocities,
                    DriveConstants.TRACK_WIDTH,
                    DriveConstants.TRACK_WIDTH,
                    1
            );
            poseVelocity = new Pose2d(poseVelocity.vec(), drive.getExternalHeadingVelocity()!=null?drive.getExternalHeadingVelocity():0);

            return poseVelocity;
        }
        return null;
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }
}
