package org.firstinspires.ftc.teamcode.PathingTool;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Peripherals;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Parameters;
import org.firstinspires.ftc.teamcode.Tools.Vector;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
public class PolarPathFollower implements Command {
    private Set<String> addedCommandKeys;
    private CommandScheduler scheduler;
    public double pathStartTime;
    private JSONArray points;
    private PID xPID = new PID(3.6, 0, 1.9);
    private PID yPID = new PID(3.6, 0, 1.9);
    private final PID yawPID = new PID(1, 0, 0);
    private HashMap<String, Supplier<Command>> commandMap;
    private HashMap<String, BooleanSupplier> conditionMap;
    private ArrayList<Command> activeCommands = new ArrayList<>();
    private double nextX, nextY;
    private Drive drive; // Adding Drive instance
    public PolarPathFollower(Drive drive, Peripherals peripherals, JSONObject pathJSON,
                             HashMap<String, Supplier<Command>> commandMap,
                             HashMap<String, BooleanSupplier> conditionMap,
                             CommandScheduler scheduler) throws JSONException {
        this.scheduler = scheduler;
        this.points = pathJSON.getJSONArray("sampled_points");
        this.commandMap = commandMap;
        this.conditionMap = conditionMap;
        this.drive = drive; // Initialize Drive instance
    }
    private double getPathTime() {
        return System.currentTimeMillis() / 1000.0;
    }
    private double getCurrentTime() {
        return getPathTime() - pathStartTime;
    }
    @Override
    public void start() {
        this.pathStartTime = getPathTime();
        try {
            JSONObject currentPoint = points.getJSONObject(0);
            nextX = currentPoint.getDouble("x");
            nextY = currentPoint.getDouble("y");
            double nextTheta = currentPoint.getDouble("angle");
            Mouse.setPosition(nextX, nextY, Math.toDegrees(nextTheta));
        } catch (JSONException e) {
            throw new RuntimeException("Error reading point data from JSON", e);
        }
        yawPID.setMinInput(-180);
        yawPID.setMinInput(180);
        yawPID.setMaxOutput(2);
        yawPID.setMinOutput(-2);
    }
    public void execute() {
        FinalPose.poseUpdate();
        double elapsedTime = getPathTime() - pathStartTime;
        int index = (int) ((elapsedTime + 0.05) / 0.01);
        if (index >= points.length()) {
            index = points.length() - 1;
        }
        try {
            JSONObject currentPoint = points.getJSONObject(index);
            nextX = currentPoint.getDouble("x");
            nextY = currentPoint.getDouble("y");
            double nextTheta = currentPoint.getDouble("angle");
            double currentX = FinalPose.x;
            double currentY = FinalPose.y;
            double currentTheta = Math.toRadians(Mouse.getTheta());
            xPID.setSetPoint(nextX);
            xPID.updatePID(currentX);
            yPID.setSetPoint(nextY);
            yPID.updatePID(currentY);
            yawPID.setSetPoint(nextTheta);
            yawPID.updatePID(currentTheta);
            Vector relativePos = new Vector(-xPID.getResult(), -yPID.getResult());
            // Pass the 'drive' instance here to autoDrive
            drive.autoDrive(relativePos, -yawPID.getResult());
            JSONArray commands = points.getJSONObject(index).optJSONArray("commands");
            if (commands != null) {
                for (int i = 0; i < commands.length(); i++) {
                    JSONObject commandJSON = commands.getJSONObject(i);
                    Command command = parseCommand(commandJSON);
                    if (command != null && !activeCommands.contains(command)) {
                        scheduler.schedule(command);
                        activeCommands.add(command);
                    }
                }
            }
            System.out.println("Vector X: " + relativePos.getI() + ", Vector Y: " + relativePos.getJ() +
                    ", Theta: " + currentTheta + ", Index: " + index);
        } catch (JSONException e) {
            throw new RuntimeException("Error reading point data from JSON", e);
        }
    }
    private Command parseCommand(JSONObject commandJSON) throws JSONException {
        if (commandJSON.has("command")) {
            return singleCommandFromJSON(commandJSON);
        } else if (commandJSON.has("parallel_command_group")) {
            return parseParallelCommandGroup(commandJSON.getJSONArray("parallel_command_group"));
        } else if (commandJSON.has("sequential_command_group")) {
            return parseSequentialCommandGroup(commandJSON.getJSONArray("sequential_command_group"));
        } else if (commandJSON.has("conditional_command")) {
            return parseConditionalCommand(commandJSON.getJSONObject("conditional_command"));
        }
        return null;
    }
    private Command singleCommandFromJSON(JSONObject commandJSON) throws JSONException {
        String commandName = commandJSON.getString("command");
        if (commandMap.containsKey(commandName)) {
            return commandMap.get(commandName).get();
        }
        return null;
    }
    private Command parseParallelCommandGroup(JSONArray commands) throws JSONException {
        ArrayList<Command> commandList = new ArrayList<>();
        for (int i = 0; i < commands.length(); i++) {
            Command command = parseCommand(commands.getJSONObject(i));
            if (command != null) {
                commandList.add(command);
            }
        }
        return new ParallelCommandGroup(scheduler, Parameters.ALL, commandList.toArray(new Command[0]));
    }
    private Command parseSequentialCommandGroup(JSONArray commands) throws JSONException {
        ArrayList<Command> commandList = new ArrayList<>();
        for (int i = 0; i < commands.length(); i++) {
            Command command = parseCommand(commands.getJSONObject(i));
            if (command != null) {
                commandList.add(command);
            }
        }
        return new SequentialCommandGroup(scheduler, commandList.toArray(new Command[0]));
    }
    private Command parseConditionalCommand(JSONObject commandJSON) throws JSONException {
        BooleanSupplier condition = conditionMap.get(commandJSON.getString("condition"));
        Command onTrue = parseCommand(commandJSON.getJSONObject("on_true"));
        Command onFalse = parseCommand(commandJSON.getJSONObject("on_false"));
        return new ConditionalCommand(onTrue, onFalse, condition);
    }
    @Override
    public void end() {
        drive.drive(0,0,0,0);
    }
    @Override
    public boolean isFinished() {
        return getCurrentTime() >= points.length() * 0.01;
    }
    @Override
    public Subsystem getRequiredSubsystem() {
        return null;
    }
}
