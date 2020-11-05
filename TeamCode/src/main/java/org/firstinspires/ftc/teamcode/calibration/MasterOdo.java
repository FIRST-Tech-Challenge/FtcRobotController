package org.firstinspires.ftc.teamcode.calibration;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.autonomous.AutoStep;
import org.firstinspires.ftc.teamcode.bots.BotAction;
import org.firstinspires.ftc.teamcode.bots.BotActionObj;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;
import org.firstinspires.ftc.teamcode.odometry.OdoBase;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.Led;

import java.io.File;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.robotcore.internal.system.AppUtil.FIRST_FOLDER;
import static org.firstinspires.ftc.teamcode.autonomous.AutoRoute.NAME_BLUE;
import static org.firstinspires.ftc.teamcode.autonomous.AutoRoute.NAME_RED;
import static org.firstinspires.ftc.teamcode.autonomous.AutoStep.NO_ACTION;

@TeleOp(name="Master Odo", group="Robot15173")
public class MasterOdo extends OdoBase {

    public static String COORDINATE = "Coordinate";

    private ArrayList<AutoRoute> routes = new ArrayList<>();
    private ArrayList<Integer> blueRoutes = new ArrayList<>();
    private ArrayList<Integer> redRoutes = new ArrayList<>();

    protected double right = 0;
    protected double left = 0;


    private int selectedTopMode = 0;
    private int selectedGoToMode = 0;
    private boolean topMode = true;
    private boolean goToMode = false;

    private boolean routeListMode = false;

    private boolean startSettingMode  = false;
    private boolean routeSettingMode  = false;
    private boolean XSettingMode = true;
    private boolean YSettingMode = false;
    private boolean speedSettingMode = false;
    private boolean strategySettingMode = false;
    private boolean waitSettingMode = false;
    private boolean desiredHeadSettingMode = false;
    private boolean routeSavingMode = false;
    private boolean actionSettingMode = false;
    private boolean directionSettingMode = false;

    private boolean dynamicDestinationMode = false;

    private boolean stopSettingMode = false;

    private boolean routeNameSettingMode = true;
    private boolean routeIndexSettingMode = false;

    private boolean manualDriveMode = false;
    private boolean coordinateSavingMode = false;

    private AutoDot newDot = new AutoDot();

    private AutoStep goToInstructions = new AutoStep();
    private AutoRoute newRoute = new AutoRoute();



    private static final int[] modesTop = new int[]{0, 1, 2, 3, 4};
    private static final String[] modeNamesTop = new String[]{"Start Position", "Go To", "Routes", "Save Route", "Manual Drive"};

    private static final int[] modesStep = new int[]{0, 1, 2, 3, 4, 5, 6, 7};
    private static final String[] modeStepName = new String[]{"Destination", "Top Speed", "Strategy", "Wait", "Continue", "Heading", "Action", "Direction"};


    private double SPEED_INCREMENT = 0.1;

    private static int HEAD_INCREMENT = 45;
    private int headIncrementValue = 1;

    private Led led = null;

    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;



    @Override
    public void runOpMode() throws InterruptedException {
        try {
            super.runOpMode();
            this.led = bot.getLights();
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
            listRoutes();

            waitForStart();


            startLocator();

            if (this.led != null){
                this.led.none();
            }

            showConfig();

            while (opModeIsActive()) {
                processCommands();
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            if (locator != null){
                locator.stop();
            }
            selectedRoute.getSteps().clear();
            selectedRoute = null;
        }
    }

    private void toggleRouteSettings(){
        if (XSettingMode){
            XSettingMode = false;
            YSettingMode = true;
        }
        else if (YSettingMode){
            YSettingMode = false;
            XSettingMode = true;
        }
    }

    private void toggleRouteNaming(){
        if (routeNameSettingMode){
            routeNameSettingMode = false;
            routeIndexSettingMode = true;
        }
        else if (routeIndexSettingMode){
            routeIndexSettingMode = false;
            routeNameSettingMode = true;
        }
    }

    private void processDriveCommands(){
        double drive = gamepad1.left_stick_y;
        double turn = 0;
        double ltrigger = gamepad1.left_trigger;
        double rtrigger = gamepad1.right_trigger;
        if (ltrigger > 0){
            turn = -ltrigger;
        }
        else if (rtrigger > 0){
            turn = rtrigger;
        }

        double strafe = gamepad1.right_stick_x;


        if (Math.abs(strafe) > 0) {
            if (strafe < 0) {
                bot.strafeRight(Math.abs(strafe));
            } else {
                bot.strafeLeft(Math.abs(strafe));
            }
        } else {
            bot.move(drive, turn);
        }
    }


    private void processCommands(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
        }

        if (gamepad1.b && routeListMode){
            //clone selected Route
            cloneSelectedRoute();
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.a && (XSettingMode || YSettingMode)){
            dynamicDestinationMode = !dynamicDestinationMode;

            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.back){

            topMode = true;
            goToMode = false;

            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.left_bumper){
            if (desiredHeadSettingMode){
                switch (headIncrementValue){
                    case 1:
                        headIncrementValue = 10;
                        break;
                    case 10:
                        headIncrementValue = HEAD_INCREMENT;
                        break;
                    case 45:
                        headIncrementValue = 1;
                        break;
                }
            }
            showConfig();
            gamepadRateLimit.reset();
        }



        if (gamepad1.dpad_left ){

            if (routeSettingMode || startSettingMode){
                toggleRouteSettings();
            }
            else if (routeSavingMode){
                toggleRouteNaming();
            }

//            MODE_VALUE = -changeIncrement();
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_right){
            if (routeSettingMode || startSettingMode){
                toggleRouteSettings();
            }
            else if (routeSavingMode){
                toggleRouteNaming();
            }

//            MODE_VALUE = changeIncrement();
            showConfig();
            gamepadRateLimit.reset();
        }

        //value adjustment
        if (gamepad1.dpad_down){
            if (routeSettingMode){
                if (dynamicDestinationMode){
                    String targetRef = goToInstructions.getTargetReference();
                    Object [] keys = coordinateFunctions.keySet().toArray();
                    int index = -1;
                    boolean found = false;
                    for (int i = 0; i < keys.length; i++){
                        if (targetRef.equals(keys[i])){
                            index = i+1;
                            found = true;
                            break;
                        }
                    }
                    if (!found){
                        index++;
                    }

                    if (index < 0 || index >= keys.length){
                        goToInstructions.setTargetReference("");
                    }
                    else{
                        goToInstructions.setTargetReference(keys[index].toString());
                    }

                }
                else if (XSettingMode) {
                    int x = goToInstructions.getTargetX();
                    x -= 5;
                    goToInstructions.setTargetX(x);
                }
                else if (YSettingMode){
                    int y = goToInstructions.getTargetY();
                    y -= 5;
                    goToInstructions.setTargetY(y);
                }

            }
            else if (startSettingMode){
                int x = (int)locator.getXInches();
                int y = (int)locator.getYInches();
                if (XSettingMode) {
                    x -= 5;
                }
                else if (YSettingMode){
                    y -= 5;
                }
                locator.init(new Point(x, y), locator.getInitialOrientation());
                goToInstructions.setTargetX(x);
                goToInstructions.setTargetY(y);
            }
            else if (speedSettingMode){
                double speed = goToInstructions.getTopSpeed();
                speed = speed - SPEED_INCREMENT;
                if (speed < 0){
                    speed = 0;
                }
                goToInstructions.setTopSpeed(speed);
            }
            else if (routeListMode){
                for(int i = 0; i < routes.size(); i++){
                    AutoRoute r = routes.get(i);
                    if (r.isSelected()){
                        r.setSelected(false);
                        AutoRoute selected = null;
                        if (i + 1 >= routes.size()){
                            selected = routes.get(0);
                            selected.setSelected(true);
                        }
                        else{
                            selected = routes.get(i+1);
                            selected.setSelected(true);
                        }
                        selectedRoute = selected;
                        break;
                    }
                }
            }
            else if (strategySettingMode){
                int index = goToInstructions.getMoveStrategy().ordinal();
                int total = MoveStrategy.values().length;
                index++;
                if (index >= total){
                    index = 0;
                }
                MoveStrategy updated = MoveStrategy.values()[index];
                goToInstructions.setMoveStrategy(updated);

            }
            else if (directionSettingMode){
                int index = goToInstructions.getRobotDirection().ordinal();
                int total = RobotDirection.values().length;
                index++;
                if (index >= total){
                    index = 0;
                }
                RobotDirection updated = RobotDirection.values()[index];
                goToInstructions.setRobotDirection(updated);
            }
            else if (desiredHeadSettingMode){
                double desiredHead = goToInstructions.getDesiredHead();
                if (desiredHead == -1){
                    desiredHead = 0;
                }
                desiredHead -= headIncrementValue;
                if (desiredHead < 0){
                    desiredHead = 0;
                }
                goToInstructions.setDesiredHead(desiredHead);
            }
            else if (waitSettingMode){
                int waitMS = goToInstructions.getWaitMS();
                waitMS -= 500;
                if (waitMS < 0){
                    waitMS = 0;
                }
                goToInstructions.setWaitMS(waitMS);
            }
            else if(stopSettingMode){
                boolean continous = goToInstructions.isContinuous();
                goToInstructions.setContinuous(!continous);
            }
            else if (actionSettingMode){
                String action = goToInstructions.getAction();
                int selectedIndex = -1;
                for(int i = 0; i < botActions.size(); i++){
                    if (botActions.get(i).getName().equals(action)){
                        selectedIndex = i;
                        break;
                    }
                }
                selectedIndex++;
                if (selectedIndex == botActions.size()){
                    selectedIndex = -1;
                }
                if (selectedIndex == -1){
                    goToInstructions.setAction(NO_ACTION);
                }
                else{
                    goToInstructions.setAction(botActions.get(selectedIndex).getName());
                }
            }
            else if (coordinateSavingMode){
                String dotName = newDot.getDotName();
                int ascii = (int) dotName.charAt(0);
                ascii--;
                if (ascii < AutoDot.asciiA){
                    ascii = AutoDot.asciiZ;
                }
                newDot.setDotName(Character.toString ((char) ascii));
            }
            else if (routeSavingMode) {
                if (routeIndexSettingMode){
                    int i = selectedRoute.getNameIndex();
                    ArrayList<Integer> list = blueRoutes;
                    if (selectedRoute.getName() == NAME_RED){
                        list = redRoutes;
                    }
                    i--;
                    while(list.contains(i)) {
                        i--;
                    }
                    if (i > 0) {
                        selectedRoute.setNameIndex(i);
                    }
                }
                else if (routeNameSettingMode){
                    String name = selectedRoute.getName();
                    if (name.equals(NAME_BLUE)){
                        if (selectedRoute.getNameIndex() == 0){
                            selectedRoute.setNameIndex(getMinAvailableIndex(redRoutes));
                        }
                        name = NAME_RED;
                    }
                    else{
                        if (selectedRoute.getNameIndex() == 0){
                            selectedRoute.setNameIndex(getMinAvailableIndex(blueRoutes));
                        }
                    }
                selectedRoute.setName(name);
                }
            }
            else{
                if (topMode) {
                    if (selectedTopMode < modesTop.length) {
                        selectedTopMode++;
                    }
                }
                else if (goToMode){
                    if (selectedGoToMode < modesStep.length) {
                        selectedGoToMode++;
                    }
                }
            }
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.dpad_up){
            if (routeSettingMode){
                if (dynamicDestinationMode){
                    String targetRef = goToInstructions.getTargetReference();
                    Object [] keys = coordinateFunctions.keySet().toArray();
                    int index = -1;
                    boolean found = false;
                    for (int i = 0; i < keys.length; i++){
                        if (targetRef.equals(keys[i])){
                            index = i-1;
                            found = true;
                            break;
                        }
                    }
                    if (!found){
                        index = keys.length -1;
                    }
                    if (index < 0 || index >= keys.length){
                        goToInstructions.setTargetReference("");
                    }
                    else{
                        goToInstructions.setTargetReference(keys[index].toString());
                    }

                }
                else if (XSettingMode) {
                    int x = goToInstructions.getTargetX();
                    x += 5;
                    goToInstructions.setTargetX(x);
                }
                else if (YSettingMode){
                    int y = goToInstructions.getTargetY();
                    y += 5;
                    goToInstructions.setTargetY(y);
                }

            }
            else if (startSettingMode){
                int x = (int)locator.getXInches();
                int y = (int)locator.getYInches();
                if (XSettingMode) {
                    x += 5;
                }
                else if (YSettingMode){
                    y += 5;
                }
                locator.init(new Point(x, y), locator.getInitialOrientation());
                goToInstructions.setTargetX(x);
                goToInstructions.setTargetY(y);
            }
            else if (speedSettingMode){
                double speed = goToInstructions.getTopSpeed();
                speed = speed + SPEED_INCREMENT;
                if (speed > 1){
                    speed = 1;
                }
                goToInstructions.setTopSpeed(speed);
            }
            else if (routeListMode){
                for(int i = routes.size() - 1; i >= 0; i--){
                    AutoRoute r = routes.get(i);
                    if (r.isSelected()){
                        r.setSelected(false);
                        AutoRoute selected = null;
                        if (i <= 0){
                            selected = routes.get(routes.size() -1 );
                            selected.setSelected(true);
                        }
                        else{
                            selected = routes.get(i -i);
                            selected.setSelected(true);
                        }
                        selectedRoute = selected;
                        break;
                    }
                }
            }
            else if (desiredHeadSettingMode){
                double desiredHead = goToInstructions.getDesiredHead();
                if (desiredHead == -1){
                    desiredHead = 0;
                }
                desiredHead += headIncrementValue;
                goToInstructions.setDesiredHead(desiredHead);
            }
            else if (waitSettingMode){
                int waitMS = goToInstructions.getWaitMS();
                waitMS += 500;
                goToInstructions.setWaitMS(waitMS);
            }
            else if(stopSettingMode){
                boolean continuous = goToInstructions.isContinuous();
                goToInstructions.setContinuous(!continuous);
            }
            else if (strategySettingMode){
                int index = goToInstructions.getMoveStrategy().ordinal();
                int total = MoveStrategy.values().length;
                index--;
                if (index < 0){
                    index = total - 1;
                }
                MoveStrategy updated = MoveStrategy.values()[index];
                goToInstructions.setMoveStrategy(updated);

            }
            else if (directionSettingMode){
                int index = goToInstructions.getRobotDirection().ordinal();
                int total = RobotDirection.values().length;
                index--;
                if (index < 0){
                    index = total - 1;
                }
                RobotDirection updated = RobotDirection.values()[index];
                goToInstructions.setRobotDirection(updated);
            }
            else if (actionSettingMode){
                String action = goToInstructions.getAction();
                int selectedIndex = -1;
                for(int i = 0; i < botActions.size(); i++){
                    if (botActions.get(i).getName().equals(action)){
                        selectedIndex = i;
                        break;
                    }
                }
                selectedIndex--;
                if (selectedIndex < -1){
                    selectedIndex = botActions.size() - 1;
                }
                if (selectedIndex == -1){
                    goToInstructions.setAction(NO_ACTION);
                }
                else{
                    goToInstructions.setAction(botActions.get(selectedIndex).getName());
                }
            }
            else if (coordinateSavingMode){
                String dotName = newDot.getDotName();
                int ascii = (int) dotName.charAt(0);
                ascii++;
                if (ascii > AutoDot.asciiZ){
                    ascii = AutoDot.asciiA;
                }
                newDot.setDotName(Character.toString ((char) ascii));
            }
            else if (routeSavingMode) {
                if (routeIndexSettingMode){
                    int i = selectedRoute.getNameIndex();
                    ArrayList<Integer> list = blueRoutes;
                    if (selectedRoute.getName() == NAME_RED){
                        list = redRoutes;
                    }
                    i++;
                    while(list.contains(i)) {
                        i++;
                    }
                    selectedRoute.setNameIndex(i);
                }
                else if (routeNameSettingMode){
                    String name = selectedRoute.getName();
                    if (name.equals(NAME_RED)){
                        if (selectedRoute.getNameIndex() == 0){
                            selectedRoute.setNameIndex(getMinAvailableIndex(blueRoutes));
                        }
                        name = NAME_BLUE;
                    }
                    else{
                        if (selectedRoute.getNameIndex() == 0){
                            selectedRoute.setNameIndex(getMinAvailableIndex(redRoutes));
                        }
                    }
                    selectedRoute.setName(name);
                    //set minimal index
                }
            }
            else{
                if (topMode) {
                    if (selectedTopMode > 0) {
                        selectedTopMode--;
                    }
                }
                else if (goToMode){
                    if (selectedGoToMode > 0) {
                        selectedGoToMode--;
                    }
                }
            }

            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.start){

            if (goToMode){
                goTo(this.goToInstructions, true, NAME_RED);
            }
            else if (routeSavingMode){
                saveRoute();
            }
            else if (routeListMode){
                runSelectedRoute();
            }
            else if (manualDriveMode){
                if (!coordinateSavingMode){
                    coordinateSavingMode = true;
                }
                else{
                    coordinateSavingMode = false;
                    saveCoordinate();
                }
            }
            showConfig();
            gamepadRateLimit.reset();
        }

        if (gamepad1.x){
            if (desiredHeadSettingMode){
                goToInstructions.setDesiredHead(BotMoveProfile.DEFAULT_HEADING);
            }
            if (routeListMode){
                deleteSelectedRoute();
            }
            showConfig();
            gamepadRateLimit.reset();
        }

        //accept
        if (gamepad1.right_bumper){
            if (topMode){
                switch (selectedTopMode){
                    case 0:
                        startSettingMode = !startSettingMode;
                        break;
                    case 1:
                        topMode = false;
                        goToMode = true;
                        break;
                    case 2:
                        routeListMode = !routeListMode;
                        break;
                    case 3:
                        routeSavingMode = !routeSavingMode;
                        break;
                    case 4:
                        manualDriveMode = !manualDriveMode;
                }
            }
            else if (goToMode){
                switch (selectedGoToMode){
                    case 0:
                        routeSettingMode = !routeSettingMode;
                        break;
                    case 1:
                        speedSettingMode = !speedSettingMode;
                        break;
                    case 2:
                        strategySettingMode = !strategySettingMode;
                        break;
                    case 3:
                        waitSettingMode = !waitSettingMode;
                        break;
                    case 4:
                        stopSettingMode = !stopSettingMode;
                        break;
                    case 5:
                        desiredHeadSettingMode = !desiredHeadSettingMode;
                        break;
                    case 6:
                        actionSettingMode = !actionSettingMode;
                        break;
                    case 7:
                        directionSettingMode = !directionSettingMode;
                        break;
                }
            }
            showConfig();
            gamepadRateLimit.reset();
        }

        // manual drive
        if (manualDriveMode){
            processDriveCommands();
            showConfig();
        }
    }


    private String getStepValue(int index){
        String val = "";
        switch (index){
            case 0:
                //destination
                val = goToInstructions.getDestination();
                break;
            case 1:
                val = goToInstructions.getTopSpeedString();
                break;
            case 2:
                val = goToInstructions.getMoveStrategyString();
                break;
            case 3:
                val = goToInstructions.getWaitString();
                break;
            case 4:
                val = goToInstructions.isContinuousAsString();
                break;
            case 5:
                val = goToInstructions.getDesiredHeadString();
                break;
            case 6:
                val = goToInstructions.getAction();
                break;
            case 7:
                val = goToInstructions.getRobotDirectionString();
                break;
        }
        return val;
    }

    private void showConfig(){
        try {
            if (routeSettingMode) {
                showTarget();
            } else if (startSettingMode) {
                showStart();
            } else if (speedSettingMode) {
                telemetry.addData("Top Speed", "%.2f", goToInstructions.getTopSpeed());
            }
            else if (desiredHeadSettingMode){
                telemetry.addData("Increment", headIncrementValue);
                showHeading();
                telemetry.addData("Desired Head", "%.2f", goToInstructions.getDesiredHead());
            }
            else if (routeListMode){
                telemetry.addData("Play", "to run selected route");
                telemetry.addData("B", "to clone");
                telemetry.addData(" ", " ");
                for (AutoRoute r : routes) {
                    String routeVal = r.getRouteName();
                    if (r.getLastRunTime() > 0){
                        routeVal = String.format("%s (%d ms)", routeVal, r.getLastRunTime());
                    }
                    if (r.isSelected()) {
                        telemetry.addData(routeVal, "*");
                    } else {
                        telemetry.addData(routeVal, " ");
                    }
                }
            }
            else if (strategySettingMode) {
                for (MoveStrategy s : MoveStrategy.values()) {
                    if (goToInstructions.getMoveStrategy().equals(s)) {
                        telemetry.addData(s.name(), "*");
                    } else {
                        telemetry.addData(s.name(), " ");
                    }
                }
            }
            else if (directionSettingMode){
                for (RobotDirection rd : RobotDirection.values()) {
                    if (goToInstructions.getRobotDirection().equals(rd)) {
                        telemetry.addData(rd.name(), "*");
                    } else {
                        telemetry.addData(rd.name(), " ");
                    }
                }
            }
            else if (routeSavingMode){
                if (routeIndexSettingMode) {
                    telemetry.addData(selectedRoute.getName(), String.format("-%d*", selectedRoute.getNameIndex()));
                }
                else if(routeNameSettingMode){
                    telemetry.addData(String.format("%s*", selectedRoute.getName()), String.format("-%d", selectedRoute.getNameIndex()));
                }
            }
            else if (manualDriveMode){
                if (coordinateSavingMode){
                    telemetry.addData("New Named Coordinate", newDot.getDotName() );
                }else {
                    telemetry.addData("Manual Drive Mode", "Use sticks to operate the robot");
                    telemetry.addData("Save coordinates", "Press Start");
                    telemetry.addData("X ", locator.getXInches());
                    telemetry.addData("Y ", locator.getYInches());
                    telemetry.addData("Orientation (Degrees)", locator.getOrientation());
                }
            }
            else if (waitSettingMode) {
                telemetry.addData("Initial Wait Time MS", goToInstructions.getWaitString());
            }
            else if (actionSettingMode){
                String selected = " ";
                if (goToInstructions.getAction().equals(NO_ACTION)){
                    selected = "*";
                }
                telemetry.addData("None", selected);
                for(Method m : this.botActions){
                    selected = "";
                    String name = m.getName();
                    String displayName = name;
                    BotAction annotation = m.getAnnotation(BotAction.class);
                    if (annotation != null){
                        displayName = annotation.displayName();
                    }
                    if(coordinateFunctions.containsKey(name)){
                        AutoDot val = coordinateFunctions.get(name);
                        if (val != null){
                            selected = val.toString();
                        }
                    }
                    if (goToInstructions.getAction().equals(name)){
                        selected = String.format("%s *", selected);
                    }
                    telemetry.addData(displayName, selected);
                }
            }
            else if (stopSettingMode){
                telemetry.addData("Continue", goToInstructions.isContinuous());
            }
            else if (topMode) {
                for (int i = 0; i < modesTop.length; i++) {
                    String selected = i == selectedTopMode ? "*" : " ";
                    telemetry.addData(selected, modeNamesTop[i]);
                }
            } else if (goToMode) {
                showHeading();
                showStart();
                for (int i = 0; i < modesStep.length; i++) {
                    String selected = i == selectedGoToMode ? "*" : " ";
                    telemetry.addData(String.format("%s%s", selected, modeStepName[i]), getStepValue(i));
                }
            }

            telemetry.update();
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }

    private void showTarget(){
        String toX = XSettingMode ? "*" : " ";
        String toY = YSettingMode ? "*" : " ";

        telemetry.addData("Target", "%d%s : %d%s", goToInstructions.getTargetX(), toX, goToInstructions.getTargetY(), toY);
        telemetry.addData("Dynamic values", "Press A");
        if (dynamicDestinationMode) {
            String targetRef = goToInstructions.getTargetReference();
            String selected = targetRef.equals("") ? "*" : " ";
            telemetry.addData("None ", selected);
            for (Map.Entry<String, AutoDot> entry : coordinateFunctions.entrySet()) {
                selected = targetRef.equals(entry.getKey()) ? "*" : " ";
                if (entry.getValue() != null){
                    selected = String.format("%s %s", entry.getValue().toString(), selected);
                }
                telemetry.addData("Result of ", String.format("%s %s", entry.getKey(), selected));
            }
        }
    }

    private void showStart(){
        String toX = XSettingMode ? "*" : " ";
        String toY = YSettingMode ? "*" : " ";

        telemetry.addData("Start", "%d%s : %d%s", (int)locator.getXInches(), toX, (int)locator.getYInches(), toY);
    }

    private void showHeading(){
        telemetry.addData("Current Heading", "%.2f", locator.getOrientation());
    }


    private void saveRoute(){
        try{
            if (selectedRoute.getSteps().size() > 0) {
                boolean newRoute = selectedRoute.isTemp();
                selectedRoute.setTemp(false);

                String name = selectedRoute.getRouteName();
                File configFile = getRouteFile(name);

                String jsonPath = selectedRoute.serialize();
                ReadWriteFile.writeFile(configFile, jsonPath);

                if (newRoute){
                    initRoute();
                    addRoute(this.newRoute);
                }
                selectedTopMode = 2;
                routeListMode = true;
                routeSavingMode = false;
                topMode = true;
            }
        }
        catch (Exception e) {
            telemetry.addData("Error", "Route cannot be saved. %s", e.getMessage());
            telemetry.update();
        }
    }

    private void cloneSelectedRoute(){
        try{
            if (selectedRoute.getSteps().size() > 0) {
                boolean newRoute = selectedRoute.isTemp();
                if (newRoute){
                    return;
                }
                AutoRoute cloned = selectedRoute.clone();
                cloned.setSelected(false);
                if (cloned.getName().equals(NAME_BLUE)){
                    cloned.setNameIndex(getMinAvailableIndex(blueRoutes));
                }
                else{
                    cloned.setNameIndex(getMinAvailableIndex(redRoutes));
                }

                String name = cloned.getRouteName();
                File configFile = getRouteFile(name);

                String jsonPath = cloned.serialize();
                ReadWriteFile.writeFile(configFile, jsonPath);

                addRoute(cloned);

                selectedTopMode = 2;
                routeListMode = true;
                routeSavingMode = false;
                topMode = true;
            }
        }
        catch (Exception e) {
            telemetry.addData("Error", "Route cannot be cloned. %s", e.getMessage());
            telemetry.update();
        }
    }

    private void saveRouteFile(AutoRoute route){
        String name = route.getRouteName();
        File configFile = getRouteFile(name);

        String jsonPath = route.serialize();
        ReadWriteFile.writeFile(configFile, jsonPath);
    }

    private void saveCoordinate(){
        try {
            String name = newDot.getDotName();
            File configFile = getCoordinateFile(name);

            newDot.setX((int) locator.getXInches());
            newDot.setY((int) locator.getYInches());
            newDot.setHeading(locator.getOrientation());

            String jsonPath = newDot.serialize();
            ReadWriteFile.writeFile(configFile, jsonPath);

            addNamedCoordinate(newDot);

            newDot = new AutoDot();
        }
        catch (Exception ex){
            telemetry.addData("Error", "Coordinate cannot be saved. %s", ex.getMessage());
            telemetry.update();
        }
    }


    private void runSelectedRoute(){
        try {
            AutoRoute selected = null;
            for (AutoRoute r : routes) {
                if (r.isSelected()) {
                    selected = r;
                    break;
                }
            }
            if (selected != null) {
                locator.init(selected.getStart(), desiredHead);
                long startTime = System.currentTimeMillis();
                for (AutoStep s : selected.getSteps()) {
                    this.goTo(s, false, selected.getName());
                }
                long endTime = System.currentTimeMillis();
                selected.setLastRunTime(endTime - startTime);
                saveRouteFile(selected);
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", "Run selected route. %s", ex.getMessage());
        }
    }

    private void deleteSelectedRoute(){
        String selectedName = "";
        for (int i  = 0; i < routes.size(); i++){
            AutoRoute r = routes.get(i);
            if (r.isTemp()){
                continue;
            }
            if (r.isSelected()){
                selectedName = r.getRouteName();
                try {
                    File f = getRouteFile(selectedName);
                    routes.remove(i);
                    f.delete();

                    int index = r.getNameIndex();
                    if (r.getName().equals(NAME_BLUE) ){
                        clearRouteCache(this.blueRoutes, index);
                    }
                    else if  (r.getName().equals(NAME_RED)){
                        clearRouteCache(this.redRoutes, index);
                    }
                    if (routes.size() > 0){
                        routes.get(0).setSelected(true);
                        selectedRoute = routes.get(0);
                    }

                }
                catch (Exception ex){
                    telemetry.addData("Error", ex.getMessage());
                    telemetry.update();
                }
                break;
            }
        }
    }

    private static void clearRouteCache(ArrayList<Integer> cache, int indexValue){
        for (int i = 0; i < cache.size(); i++){
            if (cache.get(i).equals(indexValue)){
                cache.remove(i);
                break;
            }
        }
    }

    @Override
    protected ArrayList<BotActionObj> loadBotActions() {
        ArrayList<BotActionObj> botActionList = super.loadBotActions();
        try {
            File actionsFile = getActionsFile();
            String jsonPath = SimpleGson.getInstance().toJson(botActionList);
            ReadWriteFile.writeFile(actionsFile, jsonPath);
        }
        catch (Exception ex){
            telemetry.addData("Error", "Unable to save Bot Actions. %s", ex.getMessage());
        }
        return botActionList;
    }


    private File getCoordinateFile(String filename)
    {
        String fullName = String.format("%s.json", filename);
        File file = new File(fullName);
        if (!file.isAbsolute())
        {
            AppUtil.getInstance().ensureDirectoryExists(DOTS_FOLDER);
            file = new File(DOTS_FOLDER, fullName);
        }
        return file;
    }

    private File getActionsFile()
    {
        return AppUtil.getInstance().getSettingsFile(BOT_ACTIONS);
    }

    private void initRoute(){
        if(newRoute != null){
            newRoute.getSteps().clear();
            newRoute = null;
        }
        newRoute = new AutoRoute();
        newRoute.setTemp(true);
        if (newRoute.getName().equals(NAME_BLUE)){
            newRoute.setNameIndex(getMinAvailableIndex(blueRoutes));
        }
        else{
            newRoute.setNameIndex(getMinAvailableIndex(redRoutes));
        }
        goToInstructions.setTargetX(startX);
        goToInstructions.setTargetY(startY);
    }


    protected void listRoutes(){
        try {
            int count = 0;
            AppUtil.getInstance().ensureDirectoryExists(ROUTES_FOLDER);
            File [] list = ROUTES_FOLDER.listFiles();
            if (list != null && list.length > 0) {
                for (final File rf : ROUTES_FOLDER.listFiles()) {
                    String jsonData = ReadWriteFile.readFile(rf);
                    AutoRoute route = AutoRoute.deserialize(jsonData);
                    if (count == 0) {
                        route.setSelected(true);
                        selectedRoute = route;
                    } else {
                        route.setSelected(false);
                    }
                    addRoute(route);
                    count++;
                }
            }
            //add New Route
            initRoute();
            addRoute(newRoute);
            selectedRoute = this.routes.get(0);
        }
        catch (Exception ex){
            telemetry.addData("Error listRoutes", ex.getMessage());
            telemetry.update();
        }
    }

    private void addRoute(AutoRoute route){
        this.routes.add(route);
        if (route.getName().equals(NAME_BLUE)){
            this.blueRoutes.add(route.getNameIndex());
        }
        else if  (route.getName().equals(NAME_RED)){
            this.redRoutes.add(route.getNameIndex());
        }
    }

    private int getMinAvailableIndex(ArrayList<Integer> list){
        int i = 1;
        for(int x = 0; x < list.size(); x++){
            if (list.get(x).equals(i)){
                i++;
            }
            else{
                break;
            }
        }
        return i;
    }

}
