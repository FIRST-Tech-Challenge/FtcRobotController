package org.firstinspires.ftc.teamcode.ebotsutil;

import org.firstinspires.ftc.teamcode.ebotsenums.Alliance;

public class AllianceSingleton {
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private static AllianceSingleton allianceSingleton;
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private Alliance alliance;
    private double driverFieldHeading;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private AllianceSingleton(Alliance alliance){
        this.alliance = alliance;
        if(alliance == Alliance.BLUE){
            driverFieldHeading = -90;
        } else{
            driverFieldHeading = 90;
        }
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    public static Alliance getAlliance() {
        // Null protect with default assumption of blue
        instantiateIfNull();
        return allianceSingleton.alliance;
    }

    public static double getDriverFieldHeadingDeg() {
        instantiateIfNull();
        return allianceSingleton.driverFieldHeading;
    }

    public static void setAlliance(Alliance alliance){
        allianceSingleton = new AllianceSingleton(alliance);
    }

    public static boolean isBlue() {
        instantiateIfNull();
        return allianceSingleton.alliance == Alliance.BLUE;
    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private static void instantiateIfNull(){
        if (allianceSingleton == null) {
            allianceSingleton = new AllianceSingleton(Alliance.BLUE);
        }

    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

}
