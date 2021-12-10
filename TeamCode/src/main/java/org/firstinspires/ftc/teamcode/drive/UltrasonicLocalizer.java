package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

import org.apache.commons.math3.util.CombinatoricsUtils;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.mechanism.MB1242;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class UltrasonicLocalizer implements Localizer {
    public static final double DIST_TOLERANCE = 2.0;

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

            ArrayList<Vector2d> averagablePoses = new ArrayList<>();
            ArrayList<Vector2d> positions = new ArrayList<>();

            // All the different pairs of distances we can use to calculate the position of the robot

            // Make a list of them
            // For each set, calculate the position of the robot that you could get from those
            for (int i = 0; i < 4; i++) {
                double currentTheta = theta + (Math.PI/2 * i);
                double distance = distances[i];
                double x = distance * Math.sin(currentTheta);
                double y = distance * Math.cos(currentTheta);
                if (x < -0.01) {
                    x += 144;
                }
                if (y < -0.01) {
                    y += 144;
                }
                positions.add(new Vector2d(x, y)) ;
            }

            // Get all the possible combinations of the positions
            Iterator<int[]> iterator = CombinatoricsUtils.combinationsIterator(positions.size(), 2);
            List<Vector2d[]> sets = new ArrayList<>(6);
            while (iterator.hasNext()) {
                int[] combination = iterator.next();
                Vector2d first = positions.get(combination[0]);
                Vector2d second = positions.get(combination[1]);
                sets.add(new Vector2d[]{first, second});
            }

            Pose2d previousPose = new Pose2d(72, 72, theta);
            for (Vector2d[] set : sets) {
                // For each combination, take the one which is closest to the previous pose in both x and y
                Vector2d first = set[0];
                Vector2d second = set[1];

                double firstX = first.getX();
                double firstY = first.getY();
                double secondX = second.getX();
                double secondY = second.getY();


                double firstXDistanceFromPrevious = Math.abs(firstX - previousPose.getX());
                double firstYDistanceFromPrevious = Math.abs(firstY - previousPose.getY());
                double secondXDistanceFromPrevious = Math.abs(secondX - previousPose.getX());
                double secondYDistanceFromPrevious = Math.abs(secondY - previousPose.getY());

                Vector2d newVector = new Vector2d(
                        firstXDistanceFromPrevious < secondXDistanceFromPrevious ? firstX : secondX,
                        firstYDistanceFromPrevious < secondYDistanceFromPrevious ? firstY : secondY
                );
                double poseDifference = newVector.distTo(previousPose.vec());
                // If the pose difference is too big, we don't use it, since again, it's probably wrong
                // This is the main filter that makes sure no measurements too insane get used
                // It is a bit jank though, a Kalman filter would probably be a better way to do this
                // Also, if the x or y measurements for both are the same, we don't use it
                double xDifference = Math.abs(set[0].getX() - set[1].getX());
                double yDifference = Math.abs(set[0].getY() - set[1].getY());
                if (poseDifference < DIST_TOLERANCE && xDifference > DIST_TOLERANCE * 2 && yDifference > DIST_TOLERANCE * 2) {
                    averagablePoses.add(newVector);
                }
            }
            // At the end, we average all the realistic poses we found
            if (averagablePoses.size() > 0) {
                double xAverage = 0;
                double yAverage = 0;
                for (Vector2d pose : averagablePoses) {
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
