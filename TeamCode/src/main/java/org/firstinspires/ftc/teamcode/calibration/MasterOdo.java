package org.firstinspires.ftc.teamcode.calibration;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.autonomous.AutoStep;
import org.firstinspires.ftc.teamcode.bots.BotAction;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.YellowBot;
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
public class MasterOdo extends LinearOpMode {

    protected YellowBot bot = new YellowBot();
    ElapsedTime timer = new ElapsedTime();


    public static final File ROUTES_FOLDER = new File(FIRST_FOLDER, "/routes/");
    private ArrayList<AutoRoute> routes = new ArrayList<>();
    private ArrayList<Integer> blueRoutes = new ArrayList<>();
    private ArrayList<Integer> redRoutes = new ArrayList<>();
    private List<Method> botActions = new ArrayList<Method>();
    private HashMap<String, Point> coordinateFunctions = new HashMap<>();

    protected double right = 0;
    protected double left = 0;

    protected double desiredHead = 0;


    protected int startX = 50;
    protected int startY = 15;


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

    private boolean dynamicDestinationMode = false;

    private boolean stopSettingMode = false;

    private boolean routeNameSettingMode = true;
    private boolean routeIndexSettingMode = false;


    private AutoStep goToInstructions = new AutoStep();
    private AutoRoute newRoute = new AutoRoute();
    private BotMoveProfile profileCurve = null;



    private static final int[] modesTop = new int[]{0, 1, 2, 3};
    private static final String[] modeNamesTop = new String[]{"Start Position", "Go To", "Routes", "Save"};

    private static final int[] modesStep = new int[]{0, 1, 2, 3, 4, 5, 6};
    private static final String[] modeStepName = new String[]{"Destination", "Top Speed", "Strategy", "Wait", "Continue", "Heading", "Action"};


    private double SPEED_INCREMENT = 0.1;

    private static int HEAD_INCREMENT = 45;
    private int headIncrementValue = 1;

    private Led led = null;

    Deadline gamepadRateLimit;
    private final static int GAMEPAD_LOCKOUT = 500;

    RobotCoordinatePosition locator = null;



    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            this.led = bot.getLights();
            bot.initGyro();
            bot.initCalibData();
            gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
            listRoutes();

            loadBotActions();

            initRoute();

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
            newRoute.getSteps().clear();
            newRoute = null;
        }
    }

    private void startLocator(){
        if (locator == null) {
            locator = new RobotCoordinatePosition(bot, new Point(startX, startY), desiredHead, 75);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();
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


    private void processCommands(){
        if (!gamepadRateLimit.hasExpired()) {
            return;
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
                        if (i + 1 >= routes.size()){
                            routes.get(0).setSelected(true);
                        }
                        else{
                            routes.get(i+1).setSelected(true);
                        }
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
            else if (routeSavingMode) {
                if (routeIndexSettingMode){
                    int i = newRoute.getNameIndex();
                    ArrayList<Integer> list = blueRoutes;
                    if (newRoute.getName() == NAME_RED){
                        list = redRoutes;
                    }
                    i--;
                    while(list.contains(i)) {
                        i--;
                    }
                    if (i > 0) {
                        newRoute.setNameIndex(i);
                    }
                }
                else if (routeNameSettingMode){
                    String name = newRoute.getName();
                    if (name.equals(NAME_BLUE)){
                        if (newRoute.getNameIndex() == 0){
                            newRoute.setNameIndex(getMinAvailableIndex(redRoutes));
                        }
                        name = NAME_RED;
                    }
                    else{
                        if (newRoute.getNameIndex() == 0){
                            newRoute.setNameIndex(getMinAvailableIndex(blueRoutes));
                        }
                    }
                newRoute.setName(name);
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
                        if (i <= 0){
                            routes.get(routes.size() -1 ).setSelected(true);
                        }
                        else{
                            routes.get(i -i).setSelected(true);
                        }
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
                boolean continous = goToInstructions.isContinuous();
                goToInstructions.setContinuous(!continous);
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
            else if (routeSavingMode) {
                if (routeIndexSettingMode){
                    int i = newRoute.getNameIndex();
                    ArrayList<Integer> list = blueRoutes;
                    if (newRoute.getName() == NAME_RED){
                        list = redRoutes;
                    }
                    i++;
                    while(list.contains(i)) {
                        i++;
                    }
                    newRoute.setNameIndex(i);
                }
                else if (routeNameSettingMode){
                    String name = newRoute.getName();
                    if (name.equals(NAME_RED)){
                        if (newRoute.getNameIndex() == 0){
                            newRoute.setNameIndex(getMinAvailableIndex(blueRoutes));
                        }
                        name = NAME_BLUE;
                    }
                    else{
                        if (newRoute.getNameIndex() == 0){
                            newRoute.setNameIndex(getMinAvailableIndex(redRoutes));
                        }
                    }
                    newRoute.setName(name);
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
                goTo(this.goToInstructions, true);
            }
            else if (routeSavingMode){
                saveRoute();
            }
            else if (routeListMode){
                runSelectedRoute();
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
                }
            }
            showConfig();
            gamepadRateLimit.reset();
        }
    }


    private void goTo(AutoStep instruction, boolean dryRun){
        if (dryRun && newRoute.getSteps().size() == 0){
            newRoute.setStartX((int)locator.getXInches());
            newRoute.setStartY((int)locator.getYInches());
        }
        waitToStartStep(instruction.getWaitMS());
        MoveStrategy strategy = instruction.getMoveStrategy();
        executeStep(instruction, strategy, dryRun);
        Method action = findActionMethod(instruction.getAction());
        if (action != null){
            try {
                Point result = (Point)action.invoke(this.bot);
                if (coordinateFunctions.containsKey(action.getName())){
                    coordinateFunctions.put(action.getName(), result);
                }
            }
            catch (Exception ex){
                telemetry.addData("Error", ex.getMessage());
                telemetry.update();
            }
        }
        if (dryRun) {
            newRoute.getSteps().add(instruction.clone());
        }

    }

    private void executeStep(AutoStep instruction,  MoveStrategy strategy, boolean dryRun){
        Point target = new Point(instruction.getTargetX(), instruction.getTargetY());
        if (instruction.getTargetReference().equals("") == false){
            target = coordinateFunctions.get(instruction.getTargetReference());
            if (target == null){
                telemetry.addData("Warning", String.format("No data in target reference function %s", instruction.getTargetReference()));
                return;
            }
        }
        BotMoveProfile profile = BotMoveProfile.bestRoute(bot, (int)locator.getXInches(), (int)locator.getYInches(), target,
                RobotDirection.Optimal, instruction.getTopSpeed(), strategy, instruction.getDesiredHead(), locator);
        if (profile == null){
            return;
        }
        profile.setDryRun(dryRun);
        profile.setContinuous(instruction.isContinuous());

        boolean finalSpin = true;

        switch (profile.getStrategy()){
            case Curve:
                curve(profile);
                break;
            case Spin:
            case SpinNCurve:
            case SpinNStraight:
                spin(profile);
                finalSpin = false;
                break;
            case Strafe:
                strafe(profile);
                break;
            case Straight:
                moveStraight(profile);
                break;
            case Diag:
                diag(profile);
                break;

        }
        if (profile.getNextStep() != null){
            //let the locator catch up
            if (!profile.isContinuous()) {
                sleep(locator.getThreadSleepTime());
            }
            executeStep(instruction, profile.getNextStep(), dryRun);
        }

        //set the desired heading
        if (finalSpin && profile.getNextStep() == null && instruction.getDesiredHead() != BotMoveProfile.DEFAULT_HEADING) {
            sleep(locator.getThreadSleepTime());
            BotMoveProfile profileSpin = BotMoveProfile.getFinalHeadProfile(instruction.getDesiredHead(), instruction.getTopSpeed(), locator);
            if (Math.abs(profile.getAngleChange()) < 30){
                profile.setTopSpeed(0.1);
            }
            else{
                profile.setTopSpeed(0.3);
            }
            spin(profileSpin);
        }

        telemetry.update();
    }

    private void moveStraight(BotMoveProfile profile){
        bot.moveTo(profile);
    }

    private void spin(BotMoveProfile profile){
        try {
            if (profile != null) {
                bot.spin(profile, locator);
            }
        }catch (Exception ex){
            telemetry.addData("Error spin", ex.getMessage());
        }
    }

    private void diag(BotMoveProfile profile){
        try {
            if (profile != null) {
                bot.diagTo(profile);
            }
        }catch (Exception ex){
            telemetry.addData("error diag", ex.getMessage());
        }
    }

    private void  curve(BotMoveProfile profile)
    {
        bot.curveTo(profile, locator);
    }

    private void strafe(BotMoveProfile profile){
        double distance = profile.getDistance();
        double angleChange = profile.getAngleChange();
        bot.strafeToCalib(profile.getTopSpeed(), distance, angleChange > 0, profile.getMotorReduction());
    }


    private void waitToStartStep(int MS){
        timer.reset();
        while(timer.milliseconds() < MS && opModeIsActive()){

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
                if (routes.size() > 0) {
                    for (AutoRoute r : routes) {
                        if (r.isSelected()) {
                            telemetry.addData(r.getRouteName(), "*");
                        } else {
                            telemetry.addData(r.getRouteName(), " ");
                        }
                    }
                }
                else{
                    telemetry.addData("No Routes", " ");
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
            else if (routeSavingMode){
                if (routeIndexSettingMode) {
                    telemetry.addData(newRoute.getName(), String.format("-%d*", newRoute.getNameIndex()));
                }
                else if(routeNameSettingMode){
                    telemetry.addData(String.format("%s*", newRoute.getName()), String.format("-%d", newRoute.getNameIndex()));
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
                    if(coordinateFunctions.containsKey(name)){
                        Point val = coordinateFunctions.get(name);
                        if (val != null){
                            selected = val.toString();
                        }
                    }
                    if (goToInstructions.getAction().equals(name)){
                        selected = String.format("%s *", selected);
                    }
                    telemetry.addData(m.getName(), selected);
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
                if (profileCurve != null){
                    telemetry.addData("Profile", profileCurve.toString());
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
            for (Map.Entry<String, Point> entry : coordinateFunctions.entrySet()) {
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
            if (newRoute.getSteps().size() > 0) {
                String name = newRoute.getRouteName();
                File configFile = getRouteFile(name);

                String jsonPath = newRoute.serialize();
                ReadWriteFile.writeFile(configFile, jsonPath);
                addRoute(newRoute);
                initRoute();
            }
        }
        catch (Exception e) {
            telemetry.addData("Error", "Config cannot be saved. %s", e.getMessage());
            telemetry.update();
        }
    }


    private void runSelectedRoute(){
        AutoRoute selected = null;
        for(AutoRoute r : routes){
            if (r.isSelected()){
                selected = r;
                break;
            }
        }
        if (selected != null){
            locator.init(selected.getStart(), desiredHead);
            for(AutoStep s : selected.getSteps()){
                this.goTo(s, false);
            }
        }
    }

    private void deleteSelectedRoute(){
        AutoRoute selected = null;
        String selectedName = "";
        for(AutoRoute r : routes){
            if (r.isSelected()){
                selectedName = r.getRouteName();
                selected = r;
                break;
            }
        }
        if (selected != null){
            try {
                File f = getRouteFile(selectedName);

                int index = selected.getNameIndex();
                if (selected.getName().equals(NAME_BLUE) && blueRoutes.contains(index)){
                    this.blueRoutes.remove(index);
                }
                else if  (selected.getName().equals(NAME_RED) && redRoutes.contains(index)){
                    this.redRoutes.remove(index);
                }
                routes.remove(selected);
                f.delete();
            }
            catch (Exception ex){
                telemetry.addData("Error", ex.getMessage());
                telemetry.update();
            }
        }
    }


    public  void loadBotActions() {
        Class<?> klass = this.bot.getClass();
        while (klass != Object.class) { // need to iterated thought hierarchy in order to retrieve methods from above the current instance
            // iterate though the list of methods declared in the class represented by klass variable, and add those annotated with the specified annotation
            for (final Method method : klass.getDeclaredMethods()) {
                if (method.isAnnotationPresent(BotAction.class)) {
                    botActions.add(method);

                    if (Point.class.isAssignableFrom(method.getReturnType())){
                        coordinateFunctions.put(method.getName(), null);
                    }
                }
            }
            // move to the upper class in the hierarchy in search for more methods
            klass = klass.getSuperclass();
        }
    }

    public Method findActionMethod(String actionName){
        Method selected = null;
        for(Method m : this.botActions){
            if (m.getName().equals(actionName)){
                selected = m;
                break;
            }
        }
        return selected;
    }

    private File getRouteFile(String filename)
    {
        File file = new File(filename);
        if (!file.isAbsolute())
        {
            AppUtil.getInstance().ensureDirectoryExists(ROUTES_FOLDER);
            file = new File(ROUTES_FOLDER, filename);
        }
        return file;
    }

    private void initRoute(){
        if(newRoute != null){
            newRoute.getSteps().clear();
            newRoute = null;
        }
        newRoute = new AutoRoute();
        if (newRoute.getName().equals(NAME_BLUE)){
            newRoute.setNameIndex(getMinAvailableIndex(blueRoutes));
        }
        else{
            newRoute.setNameIndex(getMinAvailableIndex(redRoutes));
        }
    }

    private void listRoutes(){
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
                    } else {
                        route.setSelected(false);
                    }
                    addRoute(route);
                    count++;
                }
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
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
