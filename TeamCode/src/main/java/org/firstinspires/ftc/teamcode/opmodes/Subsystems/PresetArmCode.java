package org.firstinspires.ftc.teamcode.opmodes.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class PresetArmCode {
    private DcMotor linearSlideMotor;
    private DcMotor armMotor;

    // Constants for motor settings
    private static final double LINEAR_SLIDE_POWER = 0.8;
    private static final double ARM_POWER = 1.0;

    // Positions in degrees (as doubles)
    private static final double INIT_DEGREES = 14.0;
    private static final double GROUND_DEGREES = 10.0;   // Default position (0 degrees)
    private static final double LOW_DEGREES = 17.0;     // Position to pick up from the ground (15 degrees)
    private static final double PICK_FROM_WALL_DEGREES = 27.5; //Was 28    // Position to pick up from the ground (15 degrees)
    private static final double HIGH_DEGREES = 68.0;    // Position to place into low basket (45 degrees)
    private static final double MAX_DEGREES = 90.0;     // Position to place into an high basket (70 degrees)

    // Formula to calculate ticks per degree
    final double ARM_TICKS_PER_DEGREE =
            145.1 // encoder ticks per rotation of the bare RS-555 motor
                    * 5.2 // gear ratio of the 5.2:1 Yellow Jacket gearbox
                    * 5.0 // external gear reduction, a 20T pinion gear driving a 100T hub-mount gear (5:1 reduction)
                    * 1 / 360.0 *2; // we want ticks per degree, not per rotation
//            (((1+(46/17))) * (1+(46/11)))

    // Pre-calculated arm positions in encoder ticks based on degrees
    private final double INIT_POSITION_TICKS = INIT_DEGREES* ARM_TICKS_PER_DEGREE;
    private final double GROUND_POSITION_TICKS = GROUND_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double LOW_POSITION_TICKS = LOW_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double PFW_POSITION_TICKS =   PICK_FROM_WALL_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double HIGH_POSITION_TICKS = HIGH_DEGREES * ARM_TICKS_PER_DEGREE;
    private final double MAX_POSITION_TICKS = MAX_DEGREES * ARM_TICKS_PER_DEGREE;

    // Fudge factor for fine control of arm adjustments
    //Larger FudgeFactor = More Jerky Movements
    private static final double FUDGE_FACTOR = 5.0;
    private double armPositionFudgeFactor = 0.0;
    private double HIGH_RUNG_POSITION = -1093;

    // Arm's current target position
    private double armTargetPosition = GROUND_POSITION_TICKS;

    public PresetArmCode(DcMotor linearSlideMotor, DcMotor armMotor) {
        this.linearSlideMotor = linearSlideMotor;
        this.armMotor = armMotor;

        // Configure motors
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setArmPosition(INIT_POSITION_TICKS);  // Set the arm to the ground position by default

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * @param gamepad The gamepad used to control the robot's subsystems
     */
    public void pickSpecimenFromWall(Gamepad logitechGamepad){

        if(logitechGamepad.right_bumper){
            setArmToWall();
        }
    }
    public void controlArmAndSlide(Gamepad gamepad) {
        // Control the linear slide motor based on left and right triggers



        if (gamepad.left_trigger > 0.1) {
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(-LINEAR_SLIDE_POWER); // Move linear slide down
        } else if (gamepad.right_trigger > 0.1) {
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlideMotor.setPower(LINEAR_SLIDE_POWER); // Move linear slide up
        } else {
           // linearSlideMotor.setPower(0); // Stop linear slide movement
            int currentPosition = linearSlideMotor.getCurrentPosition();
            linearSlideMotor.setTargetPosition(currentPosition);
            linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlideMotor.setPower(0.5); // Apply a small holding power
            //linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        // Control the arm position using D-pad inputs (up, down, left, right)
        if (gamepad.dpad_up) {
            setArmToMax();  // Place piece into the high basket
        } else if (gamepad.dpad_down) {
            setArmToGround();  // Default position, doesn't touch the ground
        } else if (gamepad.dpad_left) {
            setArmToLow();  // Position to pick up sample from the ground
        } else if (gamepad.dpad_right) {
            setArmToHigh();  // Position to place into the low basket
        }
        /*
        if (gamepad.dpad_right) {
            setSlidePosition(HIGH_RUNG_POSITION);
        }

         */
        // Adjust arm position with triggers (fine control using fudge factor)
/*
        if (gamepad.right_trigger > 0.1) {
            armPositionFudgeFactor = FUDGE_FACTOR * gamepad.right_trigger;
        } else if (gamepad.left_trigger > 0.1) {
            armPositionFudgeFactor = -FUDGE_FACTOR * gamepad.left_trigger;
        } else {
            armPositionFudgeFactor = 0.0;
        }

        // Update arm target position and set it
        armTargetPosition += armPositionFudgeFactor;
        setArmPosition(armTargetPosition);
 */
    }

    private void setArmToMax() {
        setArmPosition(MAX_POSITION_TICKS); // Lift to place piece in the higher basket
    }

    private void setArmToGround() {
        setArmPosition(GROUND_POSITION_TICKS); // Set to default position
    }

    private void setArmToWall() {
        setArmPosition(PFW_POSITION_TICKS); // Lift to place piece in the higher basket
    }

    private void setArmToLow() {
        setArmPosition(LOW_POSITION_TICKS); // Lift to pick up pieces from the ground
    }

    private void setArmToHigh() {
        setArmPosition(HIGH_POSITION_TICKS); // Lift to place piece in the high basket
    }

    private void setArmPosition(double targetPosition) {
        // Safety check to ensure position is within valid range
        if (targetPosition < GROUND_POSITION_TICKS || targetPosition > (MAX_POSITION_TICKS+5.0)) {
            targetPosition = LOW_POSITION_TICKS; // Set to low/ground position if out of range
        }

        // Convert target position in ticks (double) and set motor
        armMotor.setTargetPosition((int) targetPosition);  // Motor expects integer target position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);
    }
/*
    private void setSlidePosition(double targetPosition) {
        // Safety check to ensure position is within valid range
        if (targetPosition < GROUND_POSITION_TICKS || targetPosition > (MAX_POSITION_TICKS+5.0)) {
            targetPosition = LOW_POSITION_TICKS; // Set to low/ground position if out of range
        }

        // Convert target position in ticks (double) and set motor
        linearSlideMotor.setTargetPosition((int) targetPosition);  // Motor expects integer target position
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlideMotor.setPower(LINEAR_SLIDE_POWER);
    }
*/
    public void POSITION1 (){
        //trajectory
        //raising your arm
        //extend linear slide
        //claw stuff

    }
}
/*
package org.firstinspires.ftc.teamcode.opmodes.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class PresetArmCode {
    private DcMotor linearSlideMotor;
    private DcMotor armMotor;

    // Preset arm positions (in encoder ticks)
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation



    private final int ARM_GROUND = 50; //Make sure it isn't touching actual ground
    private final int ARM_LOW = 500; //TODO: Edit to 1500 for Low Basket
    private final int ARM_HIGH = (int)(45*ARM_TICKS_PER_DEGREE); //1500 TODO: LOW BASKET
    private final int ARM_MAX = (int)(70*ARM_TICKS_PER_DEGREE); //2000; //TODO: HIGH BASKET


//1500 = low basket.... 3000
    private static final double LinearSlide_POWER = 0.8;


    // ArmMotor power settings
    private static final double ARM_POWER = 1.0;

    public PresetArmCode(DcMotor linearSlideMotor, DcMotor armMotor) {
        this.linearSlideMotor = linearSlideMotor;
        this.armMotor = armMotor;

        //REFER TO ORIGINAL ARM CODE
        // Configure the lift motor with encoders
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setArmPosition(ARM_GROUND);

        // Brake
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void controlArmPreset(Gamepad gamepad) {

        //Preset to Ground Level (Doesn't touch the ground and pivot on the claw)

        // LinearSlide Motor Control using preset positions
        if (gamepad.left_trigger > 0.1) {
            linearSlideMotor.setPower(-LinearSlide_POWER); // Move arm down
        } else if (gamepad.right_trigger > 0.1) {
            linearSlideMotor.setPower(LinearSlide_POWER); // Move arm up
        } else {
            linearSlideMotor.setPower(0); // Stop arm movement
        }


        // Arm Motor Control using preset positions
        if (gamepad.dpad_up) {
            setArmPosition(ARM_MAX); // Lift to max position, for hanging
        } else if (gamepad.dpad_down) {
            setArmPosition(ARM_GROUND); // Lift to Lowest position, pick up blocks from submersible
        } else if (gamepad.dpad_left) {
            setArmPosition(ARM_LOW); // Lift to low position, i.e Low Basket
        } else if (gamepad.dpad_right) {
            setArmPosition(ARM_HIGH); // Lift to High position, i.e High Basket
        }
    }

    // Set Lift position using encoder ticks
    private void setArmPosition(int targetPosition) {
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ARM_POWER);
    }
}
*/