package org.firstinspires.ftc.team6220_PowerPlay;


abstract public class AutoFramework extends BaseAutonomous {
    /**
     * @param AutoSelector, controls left or right
     * @throws InterruptedException If you haven't added a config for that autonomous
     */
    public void runAuto(AutoState AutoSelector) throws InterruptedException {
        //set values here
        int driveCourse;
        int targetDistance = 11;
        int[] signalArray;

        switch (AutoSelector) {

            case LeftAutos:
                signalArray = new int[]{90, 33, 90, 11, -90, 11};
                driveCourse = -90;
                break;

            case RightAutos:
                signalArray = new int[]{90, 11, -90, 11, -90, 33};
                driveCourse = 90;
                break;
            default:
                throw new IllegalArgumentException("You need to add a new case to AutoFramework in order to run that");
        }

        // initialize motors, servos, imu, cameras, etc.
        initialize();

        // detect april tag in initialize //i added one because it looks more cooler
        int signal = 1 + detectSignal();

        // grab pre-loaded cone
        driveGrabber(Constants.GRABBER_CLOSE_POSITION);

        // sleep so grabber has time to grip cone
        sleep(1000);

        // raise slides so cone doesn't drag on tiles
        driveSlidesAutonomous(Constants.SLIDE_STOW);

        // drive forward to high junction
        driveAutonomous(0, 56);

        // raise slides to high junction height
        driveSlidesAutonomous(Constants.SLIDE_HIGH);

        // strafe right to face high junction
        driveAutonomous(driveCourse, targetDistance);

        // sleep to make sure robot has stopped moving
        sleep(500);

        // lower cone on to junction
        driveSlidesAutonomous(Constants.SLIDE_HIGH - 200);

        // sleep to make sure robot has stopped moving
        sleep(500);

        // drop cone on junction
        driveGrabber(Constants.GRABBER_OPEN_POSITION);

        // sleep to make sure cone has fallen
        sleep(500);

        // drive backward so robot is in center of junctions
        driveAutonomous(180, 3);

        // lower slides to ground
        driveSlidesAutonomous(Constants.SLIDE_BOTTOM);


        // this is not an enum because these are the value that the
        // signal detector method returns and the zones are called zones 1, 2 and 3.
        // park in correct signal position
        switch (signal) {
            // strafe to park in zone 1
            case 1:
                driveAutonomous(signalArray[0], signalArray[1]);
                break;

            // strafe to park in zone 2
            case 2:
                driveAutonomous(signalArray[2], signalArray[3]);
                break;

            // strafe to park in zone 3
            case 3:
                driveAutonomous(signalArray[4], signalArray[5]);
                break;
        }
    }

    public enum AutoState {
        LeftAutos,
        RightAutos,
    }
}
