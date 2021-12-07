package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.control.Control;
import org.firstinspires.ftc.teamcode.competition.utils.control.groups.Carfax;
import org.firstinspires.ftc.teamcode.competition.utils.control.groups.Mechanum;
import org.firstinspires.ftc.teamcode.competition.utils.control.groups.Tank;
import org.firstinspires.ftc.teamcode.competition.utils.control.items.Motor;
import org.firstinspires.ftc.teamcode.competition.utils.control.items.StandardServo;

import java.util.ArrayList;

public class InputLocation {

    private final LocationType TYPE;
    private final String NAME, ID;
    private final Control CONTROL;

    /**
     * A list of all possible locations the robot could send input to. It's highly likely the robot will not use all of these types. It's basically a list of every type of location we've ever had to have so we have a location for everything. If a location is needed, add it and don't remove it. Make sure to add it to the switch statement in the constructor.
     */
    public enum LocationType {
        RIGHT_TOP_DRIVING_MOTOR,
        RIGHT_BOTTOM_DRIVING_MOTOR,
        LEFT_TOP_DRIVING_MOTOR,
        LEFT_BOTTOM_DRIVING_MOTOR,
        CENTER_DRIVING_MOTOR,
        CARFAX_DRIVETRAIN,
        MECHANUM_DRIVETRAIN,
        TANK_DRIVETRAIN,
        DUCK_SPINNING_MOTOR,
        INTAKE_SPINNING_MOTOR,
        RIGHT_INTAKE_LIFTING_SERVO,
        LEFT_INTAKE_LIFTING_SERVO,
        TOP_INTAKE_LIFTING_SERVO,
        HAND_FIRST_DEGREE_OF_FREEDOM_SERVO,
        HAND_SECOND_DEGREE_OF_FREEDOM_SERVO,
        HAND_THIRD_DEGREE_OF_FREEDOM_SERVO,
        RIGHT_ELEVATOR_MOTOR,
        LEFT_ELEVATOR_MOTOR,
    }

    /**
     * An input location refers to a place on the robot input can be sent to. It's extremely high level and is designed to be the same across all devices which receive input, like motors, servos, etc. They hold references to wrappers of each physical device. To control a device, they take some data and infer how to send it to the device it represents based on the type of location it is, and then sends that input to the device's wrapper, which handles the input. Any exceptions thrown by the device's wrapper will be caught by the location when input is sent and will be thrown to the user inside an UnsupportedOperationException, with the details of the actual exception in its message.
     *
     * When creating a device, it will be found based on its resource name from strings.xml inferred from its type, NOT its ID. IDs are only for people to be able to target a location when the specific reference of the location is not accessible. They're like a simplified form of reflection. Know that this means each type refers to the type of only one device. If you need another device, add another type of device. Never remove a type of device.
     *
     *
     * @param hardware The hardware map of the location
     * @param type The type of location this location is. It will be used to determine which device this location refers to
     * @param id The ID of the location. This is only for ease-of-use
     * @param nestedLocations Any nested locations required by the location, for example motors required by a drivetrain. If there are none, this should be null
     * @throws NullPointerException Exception thrown when the nestedLocations list is null but shouldn't be
     * @throws ArrayIndexOutOfBoundsException Exception thrown when the nestedLocations list does not contain enough locations
     * @throws ArrayStoreException Exception thrown when the nestedLocations list does not contain the right types of locations at the right indexes
     */
    public InputLocation(HardwareMap hardware, LocationType type, String id, ArrayList<Control> nestedLocations) throws NullPointerException, ArrayIndexOutOfBoundsException, ArrayStoreException {
        ID = id;
        TYPE = type;
        // Set up locations based on their type
        switch(TYPE) {
            case RIGHT_TOP_DRIVING_MOTOR:
                NAME = hardware.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_1);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case RIGHT_BOTTOM_DRIVING_MOTOR:
                NAME = hardware.appContext.getString(R.string.DRIVETRAIN_RIGHT_DRIVE_2);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case LEFT_TOP_DRIVING_MOTOR:
                NAME = hardware.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_1);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case LEFT_BOTTOM_DRIVING_MOTOR:
                NAME = hardware.appContext.getString(R.string.DRIVETRAIN_LEFT_DRIVE_2);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case CENTER_DRIVING_MOTOR:
                NAME = hardware.appContext.getString(R.string.DRIVETRAIN_CENTER_DRIVE_1);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case DUCK_SPINNING_MOTOR:
                NAME = hardware.appContext.getString(R.string.HARDWARE_DUCK_MOTOR);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case INTAKE_SPINNING_MOTOR:
                NAME = hardware.appContext.getString(R.string.HARDWARE_INTAKE);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case RIGHT_INTAKE_LIFTING_SERVO:
                NAME = hardware.appContext.getString(R.string.HARDWARE_INTAKE_SERVO_LOWER_ONE);
                CONTROL = new StandardServo(hardware, NAME);
                break;
            case LEFT_INTAKE_LIFTING_SERVO:
                NAME = hardware.appContext.getString(R.string.HARDWARE_INTAKE_SERVO_LOWER_TWO);
                CONTROL = new StandardServo(hardware, NAME);
                break;
            case TOP_INTAKE_LIFTING_SERVO:
                NAME = hardware.appContext.getString(R.string.HARDWARE_INTAKE_SERVO_UPPER);
                CONTROL = new StandardServo(hardware, NAME);
                break;
            case HAND_FIRST_DEGREE_OF_FREEDOM_SERVO:
                NAME = hardware.appContext.getString(R.string.HARDWARE_HAND_FLIPPER_SERVO);
                CONTROL = new StandardServo(hardware, NAME);
                break;
            case HAND_SECOND_DEGREE_OF_FREEDOM_SERVO:
                NAME = hardware.appContext.getString(R.string.HARDWARE_HAND_TURNER_SERVO);
                CONTROL = new StandardServo(hardware, NAME);
                break;
            case HAND_THIRD_DEGREE_OF_FREEDOM_SERVO:
                NAME = hardware.appContext.getString(R.string.HARDWARE_HAND_GRABBER_SERVO);
                CONTROL = new StandardServo(hardware, NAME);
                break;
            case RIGHT_ELEVATOR_MOTOR:
                NAME = hardware.appContext.getString(R.string.HARDWARE_LIFT_ONE);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case LEFT_ELEVATOR_MOTOR:
                NAME = hardware.appContext.getString(R.string.HARDWARE_LIFT_TWO);
                CONTROL = new Motor(hardware, NAME, DcMotorSimple.Direction.FORWARD);
                break;
            case CARFAX_DRIVETRAIN:
                if(nestedLocations == null) {
                    throw new NullPointerException("Nested locations are needed, but they do not exist.");
                }
                if(nestedLocations.size() < 2) {
                    throw new ArrayIndexOutOfBoundsException("Carfax drivetrains need two nested locations!");
                }else if(!(nestedLocations.get(0) instanceof Motor) || !(nestedLocations.get(1) instanceof Motor)) {
                    throw new ArrayStoreException("Carfax drivetrains need their nested locations to be instances of Motors!");
                }
                NAME = "Carfax Drivetrain";
                CONTROL = new Carfax((Motor) nestedLocations.get(0), (Motor) nestedLocations.get(1));
                break;
            case MECHANUM_DRIVETRAIN:
                if(nestedLocations == null) {
                    throw new NullPointerException("Nested locations are needed, but they do not exist.");
                }
                if(nestedLocations.size() < 4) {
                    throw new ArrayIndexOutOfBoundsException("Mechanum drivetrains need two nested locations!");
                }else if(!(nestedLocations.get(0) instanceof Motor) || !(nestedLocations.get(1) instanceof Motor) || !(nestedLocations.get(2) instanceof Motor) || !(nestedLocations.get(3) instanceof Motor)) {
                    throw new ArrayStoreException("Mechanum drivetrains need their nested locations to be instances of Motors!");
                }
                NAME = "Mechanum Drivetrain";
                CONTROL = new Mechanum((Motor) nestedLocations.get(0), (Motor) nestedLocations.get(1), (Motor) nestedLocations.get(2), (Motor) nestedLocations.get(3));
                break;
            case TANK_DRIVETRAIN:
                if(nestedLocations == null) {
                    throw new NullPointerException("Nested locations are needed, but they do not exist.");
                }
                if(nestedLocations.size() < 4) {
                    throw new ArrayIndexOutOfBoundsException("Tank drivetrains need two nested locations!");
                }else if(!(nestedLocations.get(0) instanceof Motor) || !(nestedLocations.get(1) instanceof Motor) || !(nestedLocations.get(2) instanceof Motor) || !(nestedLocations.get(3) instanceof Motor)) {
                    throw new ArrayStoreException("Tank drivetrains need their nested locations to be instances of Motors!");
                }
                NAME = "Tank Drivetrain";
                CONTROL = new Tank((Motor) nestedLocations.get(0), (Motor) nestedLocations.get(1), (Motor) nestedLocations.get(2), (Motor) nestedLocations.get(3));
                break;
            default:
                NAME = "";
                CONTROL = new Control();
        }
    }

    public void handleIntegerInput() throws UnsupportedOperationException {
        // TODO: this and other input handlers
    }

    public void kill() {
        // TODO: this
    }

    public LocationType getType() {
        return TYPE;
    }

    /**
     * Runs a method of a control surface. For example, if you were controlling Foo which could do bar(), you could run Foo.bar() with Location.runMethodOfControlSurface.bar(). Yes this is technically just a getter for the control surface. Why is it not named that way? Because I wanted to make it easier to run methods on a control surface without thinking about interacting with the actual control surface, keeping true to the design of input locations.
     * @return Returns the control surface handled by the location. Don't think about it too much
     */
    public Control runMethodOfControlSurface() {
        return CONTROL;
    }

    public Control getControlSurface() {
        return CONTROL;
    }

    public String getID() {
        return ID;
    }

    public String getName() {
        return NAME;
    }

    public String toString() {
        return "ID: " + ID + "; Name: " + NAME;
    }

}
