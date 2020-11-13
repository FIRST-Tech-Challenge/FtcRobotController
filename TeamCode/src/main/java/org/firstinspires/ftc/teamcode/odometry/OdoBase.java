package org.firstinspires.ftc.teamcode.odometry;

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.autonomous.AutoRoute;
import org.firstinspires.ftc.teamcode.autonomous.AutoStep;
import org.firstinspires.ftc.teamcode.bots.BotAction;
import org.firstinspires.ftc.teamcode.bots.BotActionObj;
import org.firstinspires.ftc.teamcode.bots.BotMoveProfile;
import org.firstinspires.ftc.teamcode.bots.MoveStrategy;
import org.firstinspires.ftc.teamcode.bots.RobotDirection;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;

import java.io.File;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static org.firstinspires.ftc.robotcore.internal.system.AppUtil.FIRST_FOLDER;

public class OdoBase extends LinearOpMode {
    public static final File ROUTES_FOLDER = new File(FIRST_FOLDER, "/routes/");
    public static final File DOTS_FOLDER = new File(FIRST_FOLDER, "/dots/");

    protected double initHead = 0;
    protected int startX = 50;
    protected int startY = 15;

    public static String BOT_ACTIONS = "bot-actions.json";

    protected UltimateBot bot = new UltimateBot();
    protected AutoRoute selectedRoute = new AutoRoute();
    protected HashMap<String, AutoDot> coordinateFunctions = new HashMap<>();
    protected List<Method> botActions = new ArrayList<Method>();
    ElapsedTime timer = new ElapsedTime();

    protected RobotCoordinatePosition locator = null;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            bot.init(this, hardwareMap, telemetry);
            bot.initGyro();
            bot.initCalibData();

            listCoordinates();
            loadBotActions();
        }
        catch (Exception ex){
            throw new InterruptedException(ex.getMessage());
        }
    }

    protected void startLocator(){
        if (locator == null) {
            locator = new RobotCoordinatePosition(bot, new Point(startX, startY), initHead, 75);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();
        }
    }

    protected void runRoute(){
        try {
            if (this.selectedRoute != null) {
                locator.init(selectedRoute.getStart(), initHead);
                for (AutoStep s : selectedRoute.getSteps()) {
                    this.goTo(s, false, selectedRoute.getName());
                }
            }
        }
        catch (Exception ex){
            telemetry.addData("Error", "Run selected route. %s", ex.getMessage());
            telemetry.update();
        }
    }

    protected void goTo(AutoStep instruction, boolean dryRun, String opMode){
        if (dryRun && selectedRoute.getSteps().size() == 0){
            selectedRoute.setStartX((int)locator.getXInches());
            selectedRoute.setStartY((int)locator.getYInches());
        }
        waitToStartStep(instruction.getWaitMS());
        MoveStrategy strategy = instruction.getMoveStrategy();
        executeStep(instruction, strategy, dryRun);
        Method action = findActionMethod(instruction.getAction());
        if (action != null){
            try {
                Object result;
                if (isMethodOpModeSpecific(action)){
                    result = (AutoDot) action.invoke(this.bot, opMode);
                }
                else {
                    result = (AutoDot) action.invoke(this.bot);
                }
                if (coordinateFunctions.containsKey(action.getName())){
                    if (result instanceof AutoDot){
                        coordinateFunctions.put(action.getName(), (AutoDot)result);
                    }
                }
            }
            catch (Exception ex){
                telemetry.addData("Error", ex.getMessage());
                telemetry.update();
            }
        }
        if (dryRun) {
            selectedRoute.getSteps().add(instruction.clone());
        }

    }

    protected void waitToStartStep(int MS){
        timer.reset();
        while(timer.milliseconds() < MS && opModeIsActive()){

        }
    }

    private void executeStep(AutoStep instruction,  MoveStrategy strategy, boolean dryRun){
        Point target = new Point(instruction.getTargetX(), instruction.getTargetY());
        double desiredHead = instruction.getDesiredHead();
        String targetReference = instruction.getTargetReference();
        if (targetReference.equals("") == false){
            AutoDot dot = coordinateFunctions.get(targetReference);
            target = dot.getPoint();
            if (target == null){
                telemetry.addData("Warning", String.format("No data in target reference function %s", instruction.getTargetReference()));
                return;
            }
            desiredHead = dot.getHeading();
        }
        BotMoveProfile profile = BotMoveProfile.bestRoute(bot, (int)locator.getXInches(), (int)locator.getYInches(), target,
                instruction.getRobotDirection(), instruction.getTopSpeed(), strategy, desiredHead, locator);
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
            default: break;
        }
        if (profile.getNextStep() != null){
            //let the locator catch up
            if (!profile.isContinuous()) {
                sleep(locator.getThreadSleepTime());
            }
            executeStep(instruction, profile.getNextStep(), dryRun);
        }

        //set the desired heading
        if (finalSpin && profile.getNextStep() == null && desiredHead != BotMoveProfile.DEFAULT_HEADING) {
            sleep(locator.getThreadSleepTime());
            BotMoveProfile profileSpin = BotMoveProfile.getFinalHeadProfile(desiredHead, instruction.getTopSpeed(), locator);
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



    protected void loadRoute(String routeName){
        File routeFile = getRouteFile(routeName);
        String jsonData = ReadWriteFile.readFile(routeFile);
        AutoRoute route = AutoRoute.deserialize(jsonData);
        this.selectedRoute = route;
    }

    protected void listCoordinates(){
        try {
            int count = 0;
            AppUtil.getInstance().ensureDirectoryExists(DOTS_FOLDER);
            File [] list = DOTS_FOLDER.listFiles();
            if (list != null && list.length > 0) {
                for (final File rf : DOTS_FOLDER.listFiles()) {
                    String jsonData = ReadWriteFile.readFile(rf);
                    AutoDot dot = AutoDot.deserialize(jsonData);
                    if (count == 0) {
                        dot.setSelected(true);
                    } else {
                        dot.setSelected(false);
                    }
                    addNamedCoordinate(dot);
                    count++;
                }
            }
        }
        catch (Exception ex){
            telemetry.addData("Error listCoordinates", ex.getMessage());
            telemetry.update();
        }
    }

    protected ArrayList<BotActionObj> loadBotActions() {
        Class<?> klass = this.bot.getClass();
        ArrayList<BotActionObj> botActionList = new ArrayList<>();
        while (klass != Object.class) {
            for (final Method method : klass.getDeclaredMethods()) {
                if (method.isAnnotationPresent(BotAction.class)) {
                    botActions.add(method);
                    BotAction annotation = method.getAnnotation(BotAction.class);
                    BotActionObj obj = new BotActionObj();
                    obj.setMethodName(method.getName());
                    if (annotation != null){
                        obj.setDescription(annotation.displayName());
                    }
                    else{
                        obj.setDescription(method.getName());
                    }

                    if (AutoDot.class.isAssignableFrom(method.getReturnType())){
                        coordinateFunctions.put(method.getName(), null);
                        obj.setGeo(true);
                        String namedDot = annotation.defaultReturn();
                        if (!namedDot.isEmpty()){
                            obj.setReturnRef(namedDot);
                        }
                    }
                    botActionList.add(obj);
                }
            }
            klass = klass.getSuperclass();
        }

        return botActionList;
    }


    protected void addNamedCoordinate(AutoDot dot){
        this.coordinateFunctions.put(dot.getDotName(), dot);
        this.bot.addNamedCoordinate(dot);
    }

    protected File getRouteFile(String filename)
    {
        String fullName = String.format("%s.json", filename);
        File file = new File(fullName);
        if (!file.isAbsolute())
        {
            AppUtil.getInstance().ensureDirectoryExists(ROUTES_FOLDER);
            file = new File(ROUTES_FOLDER, fullName);
        }
        return file;
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

    // if a method has a single String parameter it is considered OpMode specific
    public static boolean isMethodOpModeSpecific(Method action){
        Class [] parTypes = action.getParameterTypes();

        return parTypes.length > 0 && parTypes[0].equals(String.class);
    }

}
