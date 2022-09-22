package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.autonroutines;

import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateCollectFreightWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateDumpFreightV2;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateEnterWarehouseAndCollectFreightWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StatePushOffAllianeHubWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateReverseToHubUsingVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateRotateToZeroDegreesVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateStrafeToAligNWallAfterCollectVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateStrafeToAlignTSEVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateStrafeToAllianceHubAfterCollectVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateStrafeToAllowTurnToAllianceHubVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateStrafeToTouchWallVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateUndoCollectTravelWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateEnterWarehouseForCollectVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateMoveToAllianceHubYWithOvertravelVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateUndoEnterWarehouseAndCollectFreightWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateUndoEnterWarehouseWithVelocityControl;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates.StateUndoOvertravelVelocityControl;

public class RoutineWarehouseWithCollect extends EbotsAutonRoutine{
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public RoutineWarehouseWithCollect(){

        itinerary.add(StateStrafeToAlignTSEVelocityControl.class);
//        itinerary.add(StateStrafeTowardWarehouseForDump.class);
        // Over-travel a bit and reverse to make sure marker is out of the way
        itinerary.add(StateMoveToAllianceHubYWithOvertravelVelocityControl.class);

        // Undo over-travel from previous state (get distance of over-travel from previous state)
        // Move bucket to target level during this move
        itinerary.add(StateUndoOvertravelVelocityControl.class);

        // move over a little to avoid bumping the hub during turn
        itinerary.add(StateStrafeToAllowTurnToAllianceHubVelocityControl.class);

        // Rotate to zero and strafe to hub
        itinerary.add(StateRotateToZeroDegreesVelocityControl.class);

        // figure out why this times out
        itinerary.add(StateReverseToHubUsingVelocityControl.class);

        itinerary.add(StateDumpFreightV2.class);

        // actual clicks traveled needs to set forwardClicksPushOff during transitional actions
        itinerary.add(StatePushOffAllianeHubWithVelocityControl.class);

        // this is used to determine Y travel  set strafeClicksCollect during transitional actions
        itinerary.add(StateStrafeToTouchWallVelocityControl.class);

//        // actual clicks traveled needs to set forwardClicksEnterWarehouse during transitional actions
//        itinerary.add(StateEnterWarehouseForCollectVelocityControl.class);
//
//        // alignment only, no need to pass click count to opMode
////        itinerary.add(StateStrafeAlignToWall.class);
//
//        // need heading control during this state to keep from veering off course set forwardClicksCollect during transitional actions
//        itinerary.add(StateCollectFreightWithVelocityControl.class);
//
//        // Must add state to back up to entry point, reverse travel during StateCollectFreightWithEncoders
//        itinerary.add(StateUndoCollectTravelWithVelocityControl.class);
//
//        // Move back to alliance hub for strafing
//        itinerary.add(StateUndoEnterWarehouseWithVelocityControl.class);

        // combine the steps for enter warehouse and collect freight
        itinerary.add(StateEnterWarehouseAndCollectFreightWithVelocityControl.class);
        itinerary.add(StateStrafeToAligNWallAfterCollectVelocityControl.class);
        itinerary.add(StateUndoEnterWarehouseAndCollectFreightWithVelocityControl.class);

        // strafe back to the hub y position previously used
        itinerary.add(StateStrafeToAllianceHubAfterCollectVelocityControl.class);
        // alignment only, no need for passing to opMode
        itinerary.add(StateReverseToHubUsingVelocityControl.class);

        itinerary.add(StateDumpFreightV2.class);
//
//        itinerary.add(StatePushOffAllianeHubWithVelocityControl.class);
        itinerary.add(StateStrafeToTouchWallVelocityControl.class);
        itinerary.add(StateEnterWarehouseForCollectVelocityControl.class);



//        From 12/14
//        itinerary.add(StateStrafeRightToAlignTSE.class);
////        itinerary.add(StateStrafeTowardWarehouseForDump.class);
//        // Over-travel a bit and reverse to make sure marker is out of the way
//        itinerary.add(StateStrafeToAllianceHubYWithOvertravel.class);
//
//        // Undo over-travel from previous state (get distance of over-travel from previous state)
//        // Move bucket to target level during this move
//        itinerary.add(StateUndoOverTravel.class);
//        // Rotate to zero and strafe to hub
//        itinerary.add(StateRotateToZeroDegreesVelocityControl.class);
//
//        // add alignment to wall here to square up
//        //itinerary.add(StateStrafeAlignToWall.class);
//
//
//
//        // figure out why this times out
//        itinerary.add(StateReverseToHubUsingVelocityControl.class);
//
//        itinerary.add(StateDumpFreightV2.class);
//
//        // actual clicks traveled needs to set forwardClicksPushOff during transitional actions
////        itinerary.add(StatePushOffAllianeHubWithVelocityControl.class);
//
//        // this is used to determine Y travel  set strafeClicksCollect during transitional actions
//        itinerary.add(StateStrafeToTouchWallVelocityControl.class);
//
//        // actual clicks traveled needs to set forwardClicksEnterWarehouse during transitional actions
//        itinerary.add(StateEnterWarehouseForCollect.class);
//
//        // alignment only, no need to pass click count to opMode
////        itinerary.add(StateStrafeAlignToWall.class);
//
//        // need heading control during this state to keep from veering off course set forwardClicksCollect during transitional actions
//        itinerary.add(StateCollectFreightWithVelocityControl.class);
//
//        // Must add state to back up to entry point, reverse travel during StateCollectFreightWithEncoders
//        itinerary.add(StateUndoCollectTravelWithVelocityControl.class);
//
//        // alignment only, no need for passing to opMode
////        itinerary.add(StateStrafeAlignToWall.class);
//
//        // Move back to alliance hub for strafing
//        itinerary.add(StateUndoEnterWarehouseWithVelocityControl.class);
//
//        // strafe back to the hub y position previously used
//        itinerary.add(StateStrafeToAllianceHubAfterCollectVelocityControl.class);
////
////        // Prepare to dump in alliance hub
////        itinerary.add(StateUndoPushOffAllianceHubVelocityControl.class);
////
//        itinerary.add(StateDumpFreightV2.class);
////
////        itinerary.add(StatePushOffAllianeHubWithVelocityControl.class);
//        itinerary.add(StateStrafeToTouchWallVelocityControl.class);
//        itinerary.add(StateEnterWarehouseForCollect.class);
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

}
