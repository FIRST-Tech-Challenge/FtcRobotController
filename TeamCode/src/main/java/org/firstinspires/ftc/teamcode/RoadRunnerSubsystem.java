package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

public class RoadRunnerSubsystem {
    protected MecanumDrive driveR;

    public Action test;
    public TrajectoryActionBuilder LOW_HomeToPixel_LEFT;
    public TrajectoryActionBuilder LOW_HomeToPixel_CENTER;
    public TrajectoryActionBuilder LOW_HomeToPixel_RIGHT;

    public TrajectoryActionBuilder HIGH_HomeToPixel_LEFT;
    public TrajectoryActionBuilder HIGH_HomeToPixel_CENTER;
    public TrajectoryActionBuilder HIGH_HomeToPixel_RIGHT;

    public TrajectoryActionBuilder LOW_ToBackdrop_INNER;
    public TrajectoryActionBuilder LOW_ToBackdrop_MID;
    public TrajectoryActionBuilder LOW_ToBackdrop_OUTER;
    public TrajectoryActionBuilder HIGH_ToBackdrop_INNER;
    public TrajectoryActionBuilder HIGH_ToBackdrop_MID;
    public TrajectoryActionBuilder HIGH_ToBackdrop_OUTER;

    public TrajectoryActionBuilder BackdropToStation_INNER;
    public TrajectoryActionBuilder StationToBackdrop_INNER;
    public TrajectoryActionBuilder BackdropToStation_OUTER;
    public TrajectoryActionBuilder StationToBackdrop_OUTER;

    public TrajectoryActionBuilder INNER_Station_OUTER;
    public TrajectoryActionBuilder INNER_Station_MID;
    public TrajectoryActionBuilder INNER_Station_INNER;
    public TrajectoryActionBuilder INNER_Station_INNER_SMALL;
    public TrajectoryActionBuilder OUTER_Station_OUTER;
    public TrajectoryActionBuilder OUTER_Station_MID;
    public TrajectoryActionBuilder OUTER_Station_INNER;

    public TrajectoryActionBuilder HIGH_ToStation;

    public TrajectoryActionBuilder INNER_ToStation_OUTER;
    public TrajectoryActionBuilder INNER_ToStation_MID;
    public TrajectoryActionBuilder INNER_ToStation_INNER;
    public TrajectoryActionBuilder OUTER_ToStation_OUTER;
    public TrajectoryActionBuilder OUTER_ToStation_MID;
    public TrajectoryActionBuilder OUTER_ToStation_INNER;

    public TrajectoryActionBuilder Parking_OUTER;
    public TrajectoryActionBuilder Parking_MID;
    public TrajectoryActionBuilder Parking_INNER;

    ////////////////////////////////////////////////////////////////////////////////////

    protected static int Tile = 24; //inch
    protected static int TileInverted = -24; //inch
    protected static int RobotX = 13; //inch
    protected static int RobotY = 14; //inch
    protected static double BackdropDistance = 6.5; //inch
    protected static double BackdropDistance_FIRST = 6; //inch
    protected static int StationDistance = 5; //inch
    protected static int StationDistance_SMALL = 4; //inch
    protected static int Invert = 1; //false

    //////////////////////////////////////////////////////////////////////////////////

    enum Alliance {
        RED,
        BLUE
    }

    enum Start {
        LOW,
        HIGH
    }

    enum Randomizer {
        LEFT,
        CENTER,
        RIGHT
    }

    enum Corridor {
        INNER,
        OUTER
    }

    enum Station {
        INNER,
        MID,
        OUTER
    }

    enum Parking {
        INNER,
        MID,
        OUTER
    }

    //////////////////////////////////////////////////////////////////////////////////

    protected Alliance alliance;
    protected Start start;
    protected Corridor corridor_backdropToStation;
    protected Corridor corridor_stationToBackdrop;
    protected Station station;
    protected Parking parking;

    //////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////// TRAJECTORY BUILD CONTRACTOR ////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////

    public RoadRunnerSubsystem(MecanumDrive drive, Alliance alliance,Start start, Corridor corridor_backdropToStation, Corridor corridor_stationToBackdrop, Station station, Parking parking)
    {
        this.driveR = drive;
        this.alliance = alliance;
        this.start = start;
        this.corridor_backdropToStation = corridor_backdropToStation;
        this.corridor_stationToBackdrop = corridor_stationToBackdrop;
        this.station = station;
        this.parking = parking;

        Invert = 1;
        if (alliance == Alliance.BLUE){
            Invert = -1;
            BackdropDistance_FIRST = 4;
        }

        ////////////////////////////////////////////////////////////////////////////////////
        //////////// SOS WHEN INVERTED : TRUE -> RIGHT LEFT PIXEL ARE INVERTED /////////////
        ////////////////////////////////////////////////////////////////////////////////////

        Vector2d leftPixel_LOW = new Vector2d((alliance == Alliance.BLUE) ?  1.25 * Tile : 1.5 * TileInverted - (RobotX/2 * Invert), 2 - (RobotY/2));
        Vector2d rightPixel_LOW = new Vector2d((alliance == Alliance.BLUE) ?  1.25 * Tile : 1.5 * TileInverted - (RobotX/2 * Invert), TileInverted + (RobotY/2) + 2);

        Vector2d leftPixel_HIGH = new Vector2d(1.25 * TileInverted - (RobotX/2), 2 * Tile - (RobotY/2));
        Vector2d rightPixel_HIGH = new Vector2d(1.25 * TileInverted - (RobotX/2), Tile + (RobotY/2));

        Pose2d backdrop = new Pose2d(1.5 * TileInverted * Invert, 2 * TileInverted + (RobotY/2) - BackdropDistance_FIRST, Math.PI/2);
        Pose2d backdrop_INNER = new Pose2d((alliance == Alliance.BLUE) ? 1.75 * Tile : 1.25 * TileInverted, 2 * TileInverted + (RobotY/2) - BackdropDistance, Math.PI/2);
        Pose2d backdrop_MID = new Pose2d(1.5 * TileInverted * Invert, 2 * TileInverted + (RobotY/2) - BackdropDistance, Math.PI/2);
        Pose2d backdrop_OUTER = new Pose2d((alliance == Alliance.BLUE) ? 1.25 * Tile : 1.75 * TileInverted, 2 * TileInverted + (RobotY/2) - BackdropDistance, Math.PI/2);

        Pose2d station_INNER = new Pose2d(TileInverted/2 + 3 * Invert, 2 * Tile, Math.PI/2);
        Pose2d station_OUTER = new Pose2d(2.5 * TileInverted * Invert, 1.5 * Tile, Math.PI/2);

        Pose2d middle_LOW = new Pose2d(1.5 * TileInverted * Invert, TileInverted/2, Math.toRadians((alliance == Alliance.BLUE) ? 0 : 180));
        Pose2d middle_HIGH = new Pose2d(1.5 * TileInverted * Invert, 1.5 * Tile, Math.toRadians(-90 * Invert));

        Pose2d station_POS_OUTER = new Pose2d(1.5 * TileInverted * Invert, 2.5 * Tile - (RobotY/2) - StationDistance, Math.PI/2);
        Pose2d station_POS_MID = new Pose2d(TileInverted * Invert, 2.5 * Tile - (RobotY/2) - StationDistance, Math.PI/2);
        Pose2d station_POS_INNER = new Pose2d(TileInverted/2 * Invert + 2, 3 * Tile - (RobotY/2) - StationDistance, Math.PI/2);

        Pose2d station_POS_INNER_SMALL = new Pose2d(TileInverted/2 * Invert+2, 2.7 * Tile - (RobotY/2) - StationDistance_SMALL, Math.PI/2);

        ////////////////////////////////////////////////////////////////////////////////////

        test = driveR.actionBuilder(driveR.pose)
                .lineToX(1.5 * Tile)
                .strafeTo(new Vector2d(1.5 * Tile, Tile/2))
                .build();

        ////////////////////////////////////////////////////////////////////////////////////

        LOW_HomeToPixel_LEFT = driveR.actionBuilder(driveR.pose)
                .setTangent(-45 * Invert)
                .splineToConstantHeading(new Vector2d((2 * TileInverted * Invert) - (RobotY/2 * Invert), TileInverted/2), Math.toRadians((alliance == Alliance.BLUE) ? 180 : 0))
                .splineTo(leftPixel_LOW, Math.PI/2)
                .lineToY(middle_LOW.position.y)
                .turnTo(Math.toRadians((alliance == Alliance.BLUE) ? 0 : 180));

        LOW_HomeToPixel_CENTER = driveR.actionBuilder(driveR.pose)
                .setTangent(-45 * Invert)
                .splineToConstantHeading(new Vector2d((2 * TileInverted * Invert) - (RobotY/2 * Invert), TileInverted/2), Math.toRadians((alliance == Alliance.BLUE) ? 180 : 0))
                .lineToX(TileInverted * Invert - (5 * Invert) - (RobotY/2 * Invert))
                .lineToX(middle_LOW.position.x);

        LOW_HomeToPixel_RIGHT = driveR.actionBuilder(driveR.pose)
                .setTangent(-45 * Invert)
                .splineToConstantHeading(new Vector2d((2 * TileInverted * Invert) - (RobotY/2 * Invert), TileInverted/2), Math.toRadians((alliance == Alliance.BLUE) ? 180 : 0))
                .splineTo(rightPixel_LOW, -(Math.PI)/2)
                .lineToY(middle_LOW.position.y)
                .turnTo(Math.toRadians((alliance == Alliance.BLUE) ? 0 : 180));

        ////////////////////////////////////////////////////////////////////////////////////

        HIGH_HomeToPixel_LEFT = driveR.actionBuilder(driveR.pose)
                .setTangent(-45)
                .splineToConstantHeading(new Vector2d((2 * TileInverted * Invert) - (RobotY/2 * Invert), 1.5 * Tile), 0)
                .splineTo(leftPixel_HIGH, Math.PI/2)
                .lineToY(middle_HIGH.position.y)
                .turnTo(Math.toRadians(180));

        HIGH_HomeToPixel_CENTER = driveR.actionBuilder(driveR.pose)
                .setTangent(-45)
                .splineToConstantHeading(new Vector2d((2 * TileInverted * Invert) - (RobotY/2 * Invert), 1.5 * Tile), 0)
                .lineToX(TileInverted * Invert - 5 - (RobotY/2 * Invert))
                .lineToX(middle_HIGH.position.x);

        HIGH_HomeToPixel_RIGHT = driveR.actionBuilder(driveR.pose)
                .setTangent(-45)
                .splineToConstantHeading(new Vector2d((2 * TileInverted * Invert) - (RobotY/2 * Invert), 1.5 * Tile), 0)
                .splineTo(rightPixel_HIGH, -(Math.PI)/2)
                .lineToY(middle_HIGH.position.y)
                .turnTo(Math.toRadians(-90));

        ////////////////////////////////////////////////////////////////////////////////////

        LOW_ToBackdrop_INNER = driveR.actionBuilder(middle_LOW)
                .splineToLinearHeading(backdrop_INNER, Math.toRadians(0));

        LOW_ToBackdrop_MID = driveR.actionBuilder(middle_LOW)
                .splineToLinearHeading(backdrop, Math.toRadians(0));

        LOW_ToBackdrop_OUTER = driveR.actionBuilder(middle_LOW)
                .splineToLinearHeading(backdrop_OUTER, Math.toRadians(0));

        HIGH_ToBackdrop_INNER = driveR.actionBuilder(middle_HIGH)
                .strafeToLinearHeading(new Vector2d(2.5 * TileInverted * Invert,1.5 * Tile), Math.toRadians(90))
                .strafeTo(new Vector2d(2.5 * TileInverted * Invert, TileInverted/2))
                .strafeToLinearHeading(backdrop_INNER.component1(), Math.PI/2);

        HIGH_ToBackdrop_MID = driveR.actionBuilder(middle_HIGH)
                .strafeTo(station_OUTER.component1())
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d(2.5 * TileInverted * Invert, TileInverted/2))
                .strafeToLinearHeading(backdrop.component1(), Math.PI/2);

        HIGH_ToBackdrop_OUTER = driveR.actionBuilder(middle_HIGH)
                .strafeToLinearHeading(new Vector2d(2.5 * TileInverted * Invert,1.5 * Tile), Math.toRadians(90))
                .strafeTo(new Vector2d(2.5 * TileInverted * Invert, TileInverted/2))
                .strafeToLinearHeading(backdrop_OUTER.component1(), Math.PI/2);

        if (corridor_backdropToStation == Corridor.INNER){
            HIGH_ToStation = driveR.actionBuilder(middle_HIGH)
                    .setTangent(45)
                    .splineToLinearHeading(station_INNER, 0);
        }else if (corridor_backdropToStation == Corridor.OUTER){
            HIGH_ToStation = driveR.actionBuilder(middle_HIGH)
                    .setTangent(90)
                    .splineToLinearHeading(station_OUTER, 0);
        }

        ////////////////////////////////////////////////////////////////////////////////////

        BackdropToStation_INNER = driveR.actionBuilder(backdrop_MID)
                .splineToLinearHeading(new Pose2d(station_INNER.position.x, 1.2 * TileInverted, Math.PI/2), Math.PI/2)
                .strafeTo(station_INNER.component1(), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-80, 90));

        StationToBackdrop_INNER = driveR.actionBuilder(station_INNER)
                .strafeTo(new Vector2d(station_INNER.position.x, 1.2 * TileInverted), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-80, 90))
                .strafeTo(backdrop_MID.component1());

        BackdropToStation_OUTER = driveR.actionBuilder(backdrop_MID)
                .strafeTo(new Vector2d(2.5 * TileInverted * Invert, TileInverted/2))
                .strafeTo(new Vector2d(2.5 * TileInverted * Invert,1.5 * Tile))
                .strafeTo(station_OUTER.component1());

        StationToBackdrop_OUTER = driveR.actionBuilder(station_OUTER)
                .strafeTo(new Vector2d(2.5 * TileInverted * Invert,1.5 * Tile))
                .strafeTo(new Vector2d(2.5 * TileInverted * Invert, TileInverted/2))
                .strafeToLinearHeading(backdrop_MID.component1(), Math.PI/2);

        ////////////////////////////////////////////////////////////////////////////////////

        INNER_Station_OUTER = driveR.actionBuilder(station_INNER)
                .strafeTo(station_POS_OUTER.component1());

        INNER_Station_MID = driveR.actionBuilder(station_INNER)
                .strafeTo(station_POS_MID.component1());

        INNER_Station_INNER = driveR.actionBuilder(station_INNER)
                .strafeTo(station_POS_INNER.component1(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 10;
                    }
                });

        INNER_Station_INNER_SMALL = driveR.actionBuilder(station_INNER)
                .strafeTo(station_POS_INNER_SMALL.component1(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 5;
                    }
                });

        OUTER_Station_OUTER = driveR.actionBuilder(station_OUTER)
                .setTangent(90)
                .splineToConstantHeading(station_POS_OUTER.component1(), 0);

        OUTER_Station_MID = driveR.actionBuilder(station_OUTER)
                .setTangent(90)
                .splineToConstantHeading(station_POS_MID.component1(), 0);

        OUTER_Station_INNER = driveR.actionBuilder(station_OUTER)
                .setTangent(90)
                .splineToConstantHeading(station_POS_INNER.component1(), 0);

        ////////////////////////////////////////////////////////////////////////////////////

        INNER_ToStation_OUTER = driveR.actionBuilder(station_POS_OUTER)
                .strafeTo(station_INNER.component1());

        INNER_ToStation_MID = driveR.actionBuilder(station_POS_MID)
                .strafeTo(station_INNER.component1());

        INNER_ToStation_INNER = driveR.actionBuilder(station_POS_INNER)
                .strafeTo(station_INNER.component1(), new VelConstraint() {
                    @Override
                    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                        return 8;
                    }
                });

        OUTER_ToStation_OUTER = driveR.actionBuilder(station_POS_OUTER)
                .splineToConstantHeading(station_OUTER.component1(), -90);

        OUTER_ToStation_MID = driveR.actionBuilder(station_POS_MID)
                .splineToConstantHeading(station_OUTER.component1(), -90);

        OUTER_ToStation_INNER = driveR.actionBuilder(station_POS_INNER)
                .splineToConstantHeading(station_OUTER.component1(), -90);

        //////////////////////////////////////////////////////////////////////////////////

        Parking_OUTER = driveR.actionBuilder(backdrop_MID)
            .setTangent(90)
            .splineToConstantHeading(new Vector2d(2.5 * TileInverted * Invert, 2 * TileInverted), -90)
            .splineToConstantHeading(new Vector2d(2.5 * TileInverted * Invert, 2.5 * TileInverted), -90);

        Parking_MID = driveR.actionBuilder(backdrop_MID)
                .strafeTo(new Vector2d(1.5 * TileInverted * Invert, 2.4 * TileInverted + (RobotY/2)));

        Parking_INNER = driveR.actionBuilder(backdrop_MID)
                .setTangent(90)
                .splineToConstantHeading(new Vector2d(TileInverted/2 * Invert, 2 * TileInverted), -90)
                .splineToConstantHeading(new Vector2d(TileInverted/2 * Invert, 2.5 * TileInverted), -90);
    }

    //////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////// STRATEGY FUNCTIONS ////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////

    public Pair<TrajectoryActionBuilder, TrajectoryActionBuilder> RobotToBackdrop(Randomizer randomizer){
        if (start == Start.LOW){
            if (randomizer == Randomizer.LEFT) {
                return new Pair<>(LOW_HomeToPixel_LEFT, LOW_ToBackdrop_INNER);
            }
            else if (randomizer == Randomizer.CENTER) {
                return new Pair<>(LOW_HomeToPixel_CENTER, LOW_ToBackdrop_MID);
            }
            else if (randomizer == Randomizer.RIGHT) {
                return new Pair<>(LOW_HomeToPixel_RIGHT, LOW_ToBackdrop_OUTER);
            }
        }
        else if (start == Start.HIGH){
            if (randomizer == Randomizer.LEFT) {
                return new Pair<>(HIGH_HomeToPixel_LEFT, HIGH_ToBackdrop_INNER);
            }
            else if (randomizer == Randomizer.CENTER) {
                return new Pair<>(HIGH_HomeToPixel_CENTER, HIGH_ToBackdrop_MID);
            }
            else if (randomizer == Randomizer.RIGHT) {
                return new Pair<>(HIGH_HomeToPixel_RIGHT, HIGH_ToBackdrop_OUTER);
            }
        }

        return new Pair<>(LOW_HomeToPixel_CENTER, LOW_ToBackdrop_MID);
    }

    public TrajectoryActionBuilder RobotBackdropToStation(){
        if (corridor_backdropToStation == Corridor.INNER){
            return BackdropToStation_INNER;
        } else if (corridor_backdropToStation == Corridor.OUTER) {
            return BackdropToStation_OUTER;
        }

        return BackdropToStation_INNER;
    }

    public Pair<TrajectoryActionBuilder, TrajectoryActionBuilder> RobotStation() {
        if (corridor_backdropToStation == Corridor.INNER) {
            if (corridor_stationToBackdrop == Corridor.INNER) {
                if (station == Station.INNER) {
                    return new Pair<>(INNER_Station_INNER, INNER_ToStation_INNER);
                }
            }
        } if (corridor_backdropToStation == Corridor.INNER) {
            if (corridor_stationToBackdrop == Corridor.INNER) {
                if (station == Station.MID) {
                    return new Pair<>(INNER_Station_MID, INNER_ToStation_MID);
                }
            }
        } if (corridor_backdropToStation == Corridor.INNER) {
            if (corridor_stationToBackdrop == Corridor.INNER) {
                if (station == Station.OUTER) {
                    return new Pair<>(INNER_Station_OUTER, INNER_ToStation_OUTER);
                }
            }
        }
        //////////////////////////////////////////////////////////////////////////////

        if (corridor_backdropToStation == Corridor.INNER) {
            if (corridor_stationToBackdrop == Corridor.OUTER) {
                if (station == Station.INNER) {
                    return new Pair<>(INNER_Station_INNER, OUTER_ToStation_INNER);
                }
            }
        } if (corridor_backdropToStation == Corridor.INNER) {
            if (corridor_stationToBackdrop == Corridor.OUTER) {
                if (station == Station.MID) {
                    return new Pair<>(INNER_Station_MID, OUTER_ToStation_MID);
                }
            }
        } if (corridor_backdropToStation == Corridor.INNER) {
            if (corridor_stationToBackdrop == Corridor.OUTER) {
                if (station == Station.OUTER) {
                    return new Pair<>(INNER_Station_OUTER, OUTER_ToStation_OUTER);
                }
            }
        }
        /////////////////////////////////////////////////////////////////////////////////

        if (corridor_backdropToStation == Corridor.OUTER) {
            if (corridor_stationToBackdrop == Corridor.OUTER) {
                if (station == Station.INNER) {
                    return new Pair<>(OUTER_Station_INNER, OUTER_ToStation_INNER);
                }
            }
        } if (corridor_backdropToStation == Corridor.OUTER) {
            if (corridor_stationToBackdrop == Corridor.OUTER) {
                if (station == Station.MID) {
                    return new Pair<>(OUTER_Station_MID, OUTER_ToStation_MID);
                }
            }
        } if (corridor_backdropToStation == Corridor.OUTER) {
            if (corridor_stationToBackdrop == Corridor.OUTER) {
                if (station == Station.OUTER) {
                    return new Pair<>(OUTER_Station_OUTER, OUTER_ToStation_OUTER);
                }
            }
        }
        /////////////////////////////////////////////////////////////////////////////////

        if (corridor_backdropToStation == Corridor.OUTER) {
            if (corridor_stationToBackdrop == Corridor.INNER) {
                if (station == Station.INNER) {
                    return new Pair<>(OUTER_Station_INNER, INNER_ToStation_INNER);
                }
            }
        } if (corridor_backdropToStation == Corridor.OUTER) {
            if (corridor_stationToBackdrop == Corridor.INNER) {
                if (station == Station.MID) {
                    return new Pair<>(OUTER_Station_MID, INNER_ToStation_MID);
                }
            }
        } if (corridor_backdropToStation == Corridor.OUTER) {
            if (corridor_stationToBackdrop == Corridor.INNER) {
                if (station == Station.OUTER) {
                    return new Pair<>(OUTER_Station_OUTER, INNER_ToStation_OUTER);
                }
            }
        }

        return new Pair<>(INNER_Station_INNER, INNER_ToStation_INNER);
    }

    public TrajectoryActionBuilder RobotStationToBackdrop(){
        if (corridor_stationToBackdrop == Corridor.INNER){
            return StationToBackdrop_INNER;
        } else if (corridor_stationToBackdrop == Corridor.OUTER){
            return StationToBackdrop_OUTER;
        }

        return StationToBackdrop_INNER;
    }

    public TrajectoryActionBuilder RobotParking(){
        if (parking == Parking.INNER){
            return Parking_INNER;
        } else if (parking == Parking.MID) {
            return Parking_MID;
        } else if (parking == Parking.OUTER){
            return Parking_OUTER;
        }

        return Parking_OUTER;
    }
}