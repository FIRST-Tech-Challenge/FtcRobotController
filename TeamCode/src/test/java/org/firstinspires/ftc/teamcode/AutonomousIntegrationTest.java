package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import org.horizon36596.zenith.AutoFile;
import org.horizon36596.zenith.AutoSource;
import org.horizon36596.zenith.Json;
import org.horizon36596.zenith.ivy.NamedCommands;
import org.horizon36596.zenith.ivy.AutoContext;
import org.horizon36596.zenith.ivy.AutoBuilder;
import org.horizon36596.zenith.ivy.ZenithRobot;
import org.horizon36596.simloop.fakehardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.*;
import org.junit.*;
import java.io.File;
import java.nio.file.Files;
import java.lang.reflect.Proxy;
import java.util.*;
import static org.junit.Assert.*;

/** Real factories/controllers with independent fake motor dynamics; no physical motion/calibration claim. */
public class AutonomousIntegrationTest {
    private long now = System.nanoTime();
    private final FakeHardwareMap map = new FakeHardwareMap();
    private final FakeDigitalChannel full = new FakeDigitalChannel();
    private final FakeExternalEncoder yawEncoder = new FakeExternalEncoder(6000, TurretSubsystem.TURRET_TICKS_PER_REV);
    private BioBuzzRobot robot;
    private CompetitionAutonomousTest.World context;
    private FakeMotor motor(String name) {
        FakeMotor motor = new FakeMotor(2800, 8, 28);
        map.register(name,motor); return motor;
    }
    private void materialHardware() {
        motor("intake_motor"); motor("transfer_motor_1"); motor("transfer_motor_2");
        map.register("distance_sensor",full); map.register("gate_servo",new FakeServo());
    }
    @Before public void setup() {
        Scheduler.reset(); NamedCommands.reset();
        materialHardware();
        for (String name : new String[]{"leftfront","rightfront","leftback","rightback"}) motor(name);
        map.register("pinpoint",new FakeGoBildaPinpoint());
        for (String name : new String[]{"turret_servo_1","turret_servo_2","turret_pitch","depollinator"})
            map.register(name,new FakeServo());
        motor("turret_motor"); map.register("turret_encoder",yawEncoder);
        robot = new BioBuzzRobot(TurretSubsystem.Alliance.RED,AutonomousProfile.FULL_ROBOT,()->now);
        robot.alliance=TurretSubsystem.Alliance.RED;
        robot.drive=new PedroDrive(map);
        robot.intake=new IntakeSubsystem(map); robot.transfer=new TransferSubsystem(map);
        robot.gate=new GateSubsystem(map,()->now*1e-9);
        robot.depollinator=new DepollinatorSubsystem(map,()->now*1e-9);
        robot.turret=new TurretSubsystem(map,false,()->now*1e-9);
        robot.shots=new ShotCoordinator(robot.turret,TeleOpFieldReference.target(robot.alliance,TurretSubsystem.HiveSide.AUDIENCE),
                ShotCoordinator.AimMode.AUTONOMOUS_ODOMETRY);
        robot.firing=robot.shots.firing();
        context=new CompetitionAutonomousTest.World();
        new AutonomousCommands(robot).register();
    }
    @After public void reset() { Scheduler.reset(); NamedCommands.reset(); }
    private Map<String,Object> args(Object... pairs) {
        Map<String,Object> result=new HashMap<>();
        for (int i=0;i<pairs.length;i+=2) result.put((String)pairs[i],pairs[i+1]);
        return result;
    }
    private Command command(String name,Object... args) { return NamedCommands.build(name,args(args),context); }
    private void set(String name,Object... args) {
        Command c=command(name,args); c.start(); assertTrue(c.done()); c.end(EndCondition.NATURALLY);
    }
    private void tick(double seconds) {
        now+=Math.round(seconds*1e9); map.updateAll(seconds); robot.turret.update(now*1e-9);
        robot.transfer.readSensors(now*1e-9);
        MotionSample pose=ControllerFixtures.pose(now*1e-9,1,-1,0,0,0,0).withReference(1);
        robot.outputs.apply(now*1e-9,pose,robot.turret,robot.shots,robot.firing,robot.gate,robot.transfer,robot.intake);
    }
    private void home() {
        robot.turret.homeTurretAndZeroEncoder(now*1e-9);
        for (int i=0;i<110;i++) tick(.02);
        assertTrue(robot.turret.getHomingStatus(),robot.turret.isHomed());
    }
    private double power(String name) { return map.get(com.qualcomm.robotcore.hardware.DcMotorEx.class,name).getPower(); }
    // Stage 1 always runs open-loop full power, including loading and the fourth-ball run-on.
    private com.qualcomm.robotcore.hardware.DcMotor.RunMode stage1Mode() {
        return map.get(com.qualcomm.robotcore.hardware.DcMotorEx.class,"transfer_motor_1").getMode();
    }
    private void assertStage1Loading() {
        assertEquals(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER,stage1Mode());
        assertEquals(1,power("transfer_motor_1"),0);
    }
    @Test public void actualRegistryMatchesCatalogAndResourceKeys() throws Exception {
        JSONObject catalog=new JSONObject(new String(Files.readAllBytes(new File("../autos/robot.json").toPath()),"UTF-8"));
        JSONArray commands=catalog.getJSONArray("commands"), conditions=catalog.getJSONArray("conditions");
        Set<String> names=new HashSet<>(), predicates=new HashSet<>();
        for (int i=0;i<commands.length();i++) {
            JSONObject spec=commands.getJSONObject(i); String name=spec.getString("name"); names.add(name);
            Map<String,Object> a=validArgs(spec);
            if (name.startsWith("pollenVision") || name.equals("collectPollen") || name.equals("limelightSetPipeline")) {
                assertThrows(name,IllegalArgumentException.class,()->NamedCommands.build(name,a,context));
                continue;
            }
            Command built=NamedCommands.build(name,a,context);
            Set<Object> expected=new HashSet<>();
            JSONArray required=spec.getJSONArray("requires");
            for (int j=0;j<required.length();j++) expected.add(required.getString(j).equals("drive") ? robot.drive : required.getString(j));
            assertEquals(name,expected,built.requirements());
        }
        for (int i=0;i<conditions.length();i++) predicates.add(conditions.getJSONObject(i).getString("name"));
        assertEquals(names,new HashSet<>(NamedCommands.commandNames()));
        assertEquals(predicates,new HashSet<>(NamedCommands.conditionNames()));
    }
    private static Set<String> keys(JSONObject object) {
        Set<String> keys=new HashSet<>();
        Iterator<String> iterator=object.keys(); while (iterator.hasNext()) keys.add(iterator.next());
        return keys;
    }
    private Map<String,Object> validArgs(JSONObject spec) throws JSONException {
        Map<String,Object> a=new HashMap<>();
        JSONObject params=spec.optJSONObject("params");
        if (params == null) return a;
        for (String key:keys(params)) {
            JSONObject p=params.getJSONObject(key);
            Object value=p.opt("default");
            if (value == null) {
                String type=p.getString("type");
                if (key.equals("preset")) value="diagnostic-example";
                else if (type.equals("number") || type.equals("integer")) value=p.optDouble("min",0);
                else if (type.equals("boolean")) value=false;
                else value=p.getJSONArray("values").getString(0);
            }
            a.put(key,value);
        }
        return a;
    }
    @Test public void numericArgumentsRejectNonFiniteWrongTypesAndBoundsDuringInit() throws Exception {
        JSONObject catalog=new JSONObject(new String(Files.readAllBytes(new File("../autos/robot.json").toPath()),"UTF-8"));
        JSONArray commands=catalog.getJSONArray("commands");
        for (int i=0;i<commands.length();i++) {
            JSONObject spec=commands.getJSONObject(i); String name=spec.getString("name");
            if (name.equals("collectPollen") || name.equals("limelightSetPipeline")) continue; // missing hardware separately checked
            JSONObject params=spec.optJSONObject("params");
            if (params == null) continue;
            for (String key:keys(params)) {
                JSONObject p=params.getJSONObject(key);
                if (!p.getString("type").matches("number|integer")) continue;
                for (Object invalid:new Object[]{Double.NaN,Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY,
                        p.getDouble("min")-1,p.getDouble("max")+1,"1",null}) {
                    Map<String,Object> a=validArgs(spec); a.put(key,invalid);
                    assertThrows(name+"."+key+"="+invalid,IllegalArgumentException.class,()->NamedCommands.build(name,a,context));
                }
            }
        }
        assertThrows(IllegalArgumentException.class,()->command("intakeOn","unexpected",1));
        assertThrows(IllegalArgumentException.class,()->command("aim","spin","true"));
        assertThrows(IllegalArgumentException.class,()->command("aim","side","UNKNOWN"));
        assertThrows(IllegalArgumentException.class,()->command("shootPreset","preset","missing","seconds",1));
    }
    @Test public void poseReseedCannotRunInParallelWithFeeding() {
        assertThrows(IllegalArgumentException.class, () -> org.horizon36596.zenith.ivy.Parallel.all(
                command("setFieldPose", "xIn",0,"yIn",0,"headingRad",0),
                command("feedManual", "seconds",1)));
    }
    @Test public void conditionWaitTimeoutAndSuccessSurviveFollowingInstantActions() {
        boolean[] predicate = {false};
        NamedCommands.registerCondition("auditReady", () -> predicate[0]);
        for (boolean ready : new boolean[]{false,true}) {
            predicate[0] = ready;
            AutoFile file = AutoFile.parse(Json.parse("{\"formatVersion\":4,\"start\":{\"pose\":{\"xIn\":0,\"yIn\":0}},\"steps\":["
                    + "{\"id\":\"nativeWait\",\"kind\":\"wait\",\"until\":\"auditReady\",\"timeoutS\":0.06},"
                    + "{\"id\":\"followup\",\"kind\":\"command\",\"name\":\"intakeOn\"}]}", "wait"),
                    "wait", Collections.emptyMap());
            robot.observeSteps(file.steps);
            AutoContext runtime = new AutoContext() {
                @Override public com.pedropathing.follower.Follower follower() { return context.follower; }
                @Override public Object drive() { return robot.drive; }
                @Override public Pose alliance(Pose p) { return p; }
                @Override public long nanoTime() { return now; }
                @Override public Command step(String id, Command body) { return robot.step(id,body); }
            };
            Command routine = AutoBuilder.build(file,runtime); Scheduler.schedule(routine);
            for (int i=0;i<20 && Scheduler.isScheduled(routine);i++) { now+=20_000_000; Scheduler.execute(); }
            assertFalse(Scheduler.isScheduled(routine));
            assertEquals(ready ? "SUCCESS" : "TIMEOUT", robot.lastOutcome());
            assertEquals(!ready, NamedCommands.condition("commandFailed").getAsBoolean());
        }
    }
    @Test public void terminalFailureSurvivesInterruptedCommandCleanup() {
        Command loading = command("loadUntilFull","seconds",1); loading.start();
        robot.terminalOutcome("TIMEOUT","Global autonomous deadline");
        loading.end(com.pedropathing.ivy.behaviors.EndCondition.INTERRUPTED);
        assertEquals("TIMEOUT",robot.lastOutcome());
        assertFalse(robot.outputs.loading()); assertEquals(0,power("intake_motor"),0);
    }
    @Test public void pollenProfileInitializesWithoutTurretLimelightOrDepollinator() {
        FakeHardwareMap pollenMap=new FakeHardwareMap();
        for (String name:new String[]{"leftfront","rightfront","leftback","rightback","intake_motor","transfer_motor_1","transfer_motor_2"})
            pollenMap.register(name,new FakeMotor(2800,8,28));
        pollenMap.register("pinpoint",new FakeGoBildaPinpoint()); pollenMap.register("distance_sensor",new FakeDigitalChannel());
        pollenMap.register("gate_servo",new FakeServo());
        Harness op=new Harness(); op.hardwareMap=pollenMap;
        BioBuzzRobot pollen=BioBuzzRobot.pollenTest(); pollen.init(op); NamedCommands.reset(); pollen.registerCommands(op);
        assertNull(pollen.turret); assertNull(pollen.depollinator);
        pollenMap.get(FakeGoBildaPinpoint.class,"pinpoint").update(1);
        pollen.setStartPose(Pose.zero()); pollen.drive.readSensors(); pollen.drive.readSensors();
        assertFalse("Local pollen seed must not grant an absolute field reference",pollen.drive.sample().fieldReferenced);
        assertTrue(pollen.drive.ready(System.nanoTime()*1e-9));
        assertThrows(IllegalArgumentException.class,()->command("turretSetYaw","yawDeg",0));
        assertThrows(IllegalArgumentException.class,()->command("depollinatorDeploy"));
        assertThrows(IllegalArgumentException.class,()->command("collectPollen"));
        command("stopAll"); command("feedManual","seconds",1);
        pollen.onStop(op);
    }
    @Test public void explicitTargetsSurviveBackgroundAimAndStopRetainsYawTracking() {
        home(); set("aim"); tick(.02);
        set("turretSetYaw","yawDeg",25); tick(.02);
        assertEquals(25,robot.turret.getTargetYawDegrees(),0);
        assertEquals(AutonomousOutputCoordinator.Mode.AUTO,robot.outputs.hoodMode());
        set("hoodSet","travelDeg",12); set("flywheelSetRpm","rpm",2500);
        for (int i=0;i<50;i++) tick(.02);
        assertEquals(25,robot.turret.getTargetYawDegrees(),0);
        assertEquals(12,robot.turret.getTargetPitchDegrees(),0);
        assertEquals(2500,robot.turret.getTargetShooterRpm(),0);
        set("aim"); set("flywheelStop"); tick(.02);
        assertEquals(AutonomousOutputCoordinator.Mode.AUTO,robot.outputs.yawMode());
        assertEquals(AutonomousOutputCoordinator.Mode.AUTO,robot.outputs.hoodMode());
        assertEquals(0,robot.turret.getTargetShooterRpm(),0);
        assertFalse(robot.transfer.isFeeding());
    }
    @Test public void relativeAdjustmentsResolveOnceAndRejectUnreachableTargets() {
        set("turretSetYaw","yawDeg",20);
        Command adjust=command("turretAdjustYaw","deltaDeg",5); adjust.start();
        for (int i=0;i<10;i++) { adjust.execute(); tick(.02); }
        assertEquals(25,robot.turret.getTargetYawDegrees(),0);
        set("hoodSet","travelDeg",25);
        assertThrows(IllegalArgumentException.class,()->command("hoodAdjust","deltaDeg",6).start());
        assertEquals(25,robot.turret.getTargetPitchDegrees(),0);
    }
    @Test public void manualFeedWaitsForGateAndAlwaysStopsBothStages() {
        Command feed=command("feedManual","seconds",1); feed.start(); tick(0);
        assertTrue(robot.gate.isOpenCommanded()); assertFalse(robot.transfer.isFeeding());
        tick(.099); assertFalse(robot.transfer.isFeeding()); assertEquals(0,power("transfer_motor_2"),0);
        tick(.002); assertTrue(robot.transfer.isFeeding());
        // Both stages start together once the gate delay has elapsed.
        assertEquals(1,power("transfer_motor_1"),0); assertEquals(1,power("transfer_motor_2"),0);
        tick(.05);
        assertEquals(1,power("transfer_motor_1"),0); assertEquals(1,power("transfer_motor_2"),0);
        assertEquals(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER,stage1Mode());
        feed.end(EndCondition.INTERRUPTED);
        assertFalse(robot.gate.isOpenCommanded()); assertEquals(0,power("transfer_motor_1"),0); assertEquals(0,power("transfer_motor_2"),0);
        assertTrue(robot.commandFailed());
        assertEquals("CANCELLED",robot.lastOutcome());
    }
    @Test public void disjointManualSetpointsDoNotCancelExplicitManualFeeding() {
        Command feed=command("feedManual","seconds",1); feed.start(); tick(0); tick(.201);
        assertTrue(robot.transfer.isFeeding());
        set("turretSetYaw","yawDeg",12); set("hoodSet","travelDeg",4); set("flywheelSetRpm","rpm",2000);
        tick(.02);
        assertTrue(robot.transfer.isFeeding()); assertTrue(robot.gate.isOpenCommanded());
        feed.end(EndCondition.INTERRUPTED); assertFalse(robot.gate.isOpenCommanded());
    }
    @Test public void confirmedFullSurvivesPauseAndAllowsReverseUntilActualFeeding() {
        full.setSimulatedInputState(true); set("loadOn"); set("intakeOn");
        tick(0); tick(.499); assertFalse(robot.transfer.isFull());
        tick(.002); assertTrue(robot.transfer.isFull()); assertEquals(0,power("intake_motor"),0);
        assertStage1Loading(); // Intake stops; stage 1 runs on at full power to seat the ball.
        tick(.498); assertStage1Loading(); assertEquals(0,power("intake_motor"),0);
        tick(.003); assertEquals(0,power("transfer_motor_1"),0); assertEquals(0,power("intake_motor"),0);
        set("transferStop"); full.setSimulatedInputState(false); set("loadOn"); tick(.02);
        assertFalse(robot.transfer.isFull()); assertStage1Loading();
        assertEquals(1,power("intake_motor"),0);
        set("intakeReverse"); tick(.02); assertEquals(-1,power("intake_motor"),0);
        Command feed=command("feedManual","seconds",1); feed.start(); tick(0); tick(.201);
        assertFalse(robot.transfer.isFull()); feed.end(EndCondition.NATURALLY);
    }
    @Test public void inhibitDominatesManualAndGateOnlyAndResumeDoesNotFeed() {
        set("feedInhibit");
        Command feed=command("feedManual","seconds",1); feed.start(); tick(.5);
        assertFalse(robot.gate.isOpenCommanded()); assertFalse(robot.transfer.isFeeding()); feed.end(EndCondition.INTERRUPTED);
        Command gate=command("gateOpen","seconds",1); gate.start(); tick(.5);
        assertFalse(robot.gate.isOpenCommanded()); gate.end(EndCondition.INTERRUPTED);
        set("feedResume"); tick(.5);
        assertFalse(robot.transfer.isFeeding()); assertFalse(robot.gate.isOpenCommanded());
    }
    @Test public void staleTimedCleanupCannotStopReplacementAndRunsOnce() {
        Command old=command("intakeRun","power",1,"seconds",1); old.start(); tick(.02);
        set("intakeSet","power",-.4); old.end(EndCondition.INTERRUPTED); old.end(EndCondition.INTERRUPTED); tick(.02);
        assertEquals(-.4,power("intake_motor"),0);
        Command gate=command("gateOpen","seconds",1); gate.start();
        set("loadOn"); gate.end(EndCondition.INTERRUPTED); tick(.02);
        assertTrue(robot.outputs.loading());
    }
    @Test public void flywheelSpinupRetainsOnlyNaturalSuccessAndStopsOnCancelOrTimeout() {
        Command spin=command("flywheelSpinUp","rpm",2500,"timeoutSeconds",2); spin.start();
        for (int i=0;i<100 && !spin.done();i++) tick(.02);
        assertTrue("Independent fake motor must reach speed before timeout",robot.turret.isShooterAtSpeed());
        spin.end(EndCondition.NATURALLY); tick(.02);
        assertEquals(2500,robot.turret.getTargetShooterRpm(),0);
        Command cancelled=command("flywheelSpinUp","rpm",2500,"timeoutSeconds",2); cancelled.start();
        cancelled.end(EndCondition.INTERRUPTED); tick(.02); assertEquals(0,robot.turret.getTargetShooterRpm(),0);
        Command timeout=command("flywheelSpinUp","rpm",3500,"timeoutSeconds",.001); timeout.start(); tick(.002);
        assertTrue(timeout.done()); timeout.end(EndCondition.NATURALLY);
        assertEquals(0,robot.turret.getTargetShooterRpm(),0); assertTrue(robot.commandFailed());
    }
    @Test public void unmeasuredPresetNeverAutomaticallyFeedsAndOutcomeSurvivesInstantSteps() {
        home();
        Command shot=command("shootPreset","preset","diagnostic-example","seconds",1); shot.start();
        for (int i=0;i<60;i++) { tick(.02); assertFalse(robot.transfer.isFeeding()); }
        assertTrue(shot.done()); shot.end(EndCondition.NATURALLY);
        assertTrue(robot.commandFailed()); set("intakeOff"); assertTrue(robot.commandFailed());
        assertEquals("INHIBITED",robot.lastOutcome());
        assertFalse(NamedCommands.condition("shotReady").getAsBoolean());
        assertFalse(robot.gate.isOpenCommanded());
    }
    @Test public void boundedHomeTimeoutInvalidatesIncompleteHomeAndStopAllPreservesValidHomeAndPosture() {
        Command home=command("turretHome","timeoutSeconds",.1); home.start(); tick(.11);
        assertTrue(home.done()); home.end(EndCondition.NATURALLY); assertFalse(robot.turret.isHomed());
        assertEquals("TIMEOUT",robot.lastOutcome());
        home(); set("depollinatorDeploy"); set("flywheelSetRpm","rpm",2000); tick(.02);
        set("stopAll"); tick(.02);
        assertTrue(robot.turret.isHomed()); assertTrue(robot.depollinator.isDeployedCommanded());
        assertEquals(0,robot.turret.getTargetShooterRpm(),0);
        set("flywheelSetRpm","rpm",1500); tick(.02); assertEquals(1500,robot.turret.getTargetShooterRpm(),0);
    }
    @Test public void targetFrameTransformsOnlyAuthoredSide() {
        for (TurretSubsystem.HiveSide side:TurretSubsystem.HiveSide.values()) {
            assertEquals(side,AutonomousCommands.transformSide(side,AutonomousCommands.TargetFrame.FIELD,Math.PI));
            assertEquals(side,AutonomousCommands.transformSide(side,AutonomousCommands.TargetFrame.AUTHORED,0));
            assertNotEquals(side,AutonomousCommands.transformSide(side,AutonomousCommands.TargetFrame.AUTHORED,Math.PI));
        }
    }
    @Test public void exceptionDuringShutdownPreservesOriginalError() {
        BioBuzzAutoFromFile failing=new BioBuzzAutoFromFile() {
            @Override protected String autoName() { return "failure"; }
            @Override protected ZenithRobot createRobot() { return robot; }
            @Override public void initialize() { throw new IllegalStateException("original"); }
            @Override public void reset() { throw new IllegalStateException("cleanup"); }
        };
        IllegalStateException error=assertThrows(IllegalStateException.class,failing::runOpMode);
        assertEquals("original",error.getMessage()); assertEquals("cleanup",error.getSuppressed()[0].getMessage());
    }
    @Test public void realDriveHoldAndStopLeaveNoLaterFollowerOutput() {
        Command hold=command("driveHold","xIn",12,"yIn",0,"seconds",1);
        hold.start(); robot.follower().update(.02);
        assertEquals(com.pedropathing.follower.Follower.Mode.HOLD,robot.follower().mode());
        assertTrue(power("leftfront")+power("rightfront")+power("leftback")+power("rightback") > 0);
        set("driveStop"); robot.follower().update(.02);
        assertEquals(com.pedropathing.follower.Follower.Mode.IDLE,robot.follower().mode());
        for (String name:new String[]{"leftfront","rightfront","leftback","rightback"}) assertEquals(0,power(name),0);
        hold.end(EndCondition.INTERRUPTED);
    }
    @Test public void knownPoseArgumentsMirrorOnceAndRequireANewSensorSample() {
        map.get(FakeGoBildaPinpoint.class,"pinpoint").update(1); robot.drive.readSensors();
        AutoContext mirrored=new AutoContext() {
            @Override public long nanoTime() { return now; }
            @Override public com.pedropathing.follower.Follower follower() { return robot.follower(); }
            @Override public Object drive() { return robot.drive; }
            @Override public Pose alliance(Pose p) { return robot.mirror(p); }
        };
        NamedCommands.build("setFieldPose",args("xIn",12,"yIn",-24,"headingRad",.2),mirrored).start();
        assertEquals(-12,robot.follower().pose().x(),1e-8); assertEquals(24,robot.follower().pose().y(),1e-8);
        assertEquals(.2+Math.PI,robot.follower().pose().heading(),1e-8);
        assertFalse(robot.drive.ready(System.nanoTime()*1e-9));
        int reads=map.get(FakeGoBildaPinpoint.class,"pinpoint").busReadCount();
        robot.drive.readSensors();
        assertEquals(reads+1,map.get(FakeGoBildaPinpoint.class,"pinpoint").busReadCount());
        assertTrue(robot.drive.sample().fieldReferenced); assertTrue(robot.drive.ready(System.nanoTime()*1e-9));
        robot.follower().update(.02);
        NamedCommands.condition("localizationReady").getAsBoolean();
        assertEquals(reads+1,map.get(FakeGoBildaPinpoint.class,"pinpoint").busReadCount());
        NamedCommands.build("selectTarget",args("side","AUDIENCE","frame","AUTHORED"),mirrored).start();
        assertEquals(TurretSubsystem.HiveSide.FAR,robot.turret.getActiveHiveSide());
        NamedCommands.build("selectTarget",args("side","AUDIENCE","frame","FIELD"),mirrored).start();
        assertEquals(TurretSubsystem.HiveSide.AUDIENCE,robot.turret.getActiveHiveSide());
    }
    @Test public void canonicalExampleExecutesRealFactoriesThroughNativeBuilder() throws Exception {
        AutoFile file=AutoFile.load(AutoSource.directory(new File("../autos")),"integration-example");
        AutoContext runtime=new AutoContext() {
            @Override public com.pedropathing.follower.Follower follower() { return context.follower; }
            @Override public Object drive() { return robot.drive; }
            @Override public Pose alliance(Pose p) { return p; }
            @Override public long nanoTime() { return now; }
            @Override public Command step(String id,Command command) { return robot.step(id,command); }
        };
        Command routine=AutoBuilder.build(file,runtime); Scheduler.schedule(routine);
        for (int i=0;i<1400 && Scheduler.isScheduled(routine);i++) {
            now+=20_000_000; map.updateAll(.02); robot.turret.update(now*1e-9); robot.transfer.readSensors(now*1e-9);
            context.now=now; context.advance(); Scheduler.execute();
            robot.outputs.apply(now*1e-9,ControllerFixtures.pose(now*1e-9,1,-1,0,0,0,0).withReference(1),
                    robot.turret,robot.shots,robot.firing,robot.gate,robot.transfer,robot.intake);
        }
        assertFalse(Scheduler.isScheduled(routine)); assertEquals(1,context.followed.size());
        assertFalse(robot.gate.isOpenCommanded()); assertFalse(robot.transfer.isFeeding());
        assertEquals(0,power("intake_motor"),0); assertEquals(0,robot.turret.getTargetShooterRpm(),0);
        assertFalse(robot.depollinator.isDeployedCommanded());
    }
    @Test public void cleanupFaultAttemptsRemainingOutputsAndRecordsFault() {
        FakeHardwareMap faultMap=new FakeHardwareMap(); boolean[] fail={false};
        FakeMotor first=new FakeMotor(2800,8,28) {
            @Override public void setPower(double power) {
                if (fail[0] && power == 0) throw new IllegalStateException("disconnected stage 1");
                super.setPower(power);
            }
        };
        FakeMotor second=new FakeMotor(2800,8,28);
        faultMap.register("transfer_motor_1",first); faultMap.register("transfer_motor_2",second);
        faultMap.register("distance_sensor",new FakeDigitalChannel());
        robot.transfer=new TransferSubsystem(faultMap);
        Command loading=command("loadUntilFull","seconds",1); loading.start();
        robot.transfer.update(now*1e-9,true,false); fail[0]=true;
        assertThrows(IllegalStateException.class,()->loading.end(EndCondition.INTERRUPTED));
        assertEquals(0,second.getPower(),0); assertTrue(robot.commandFailed());
        assertEquals("FAULT",robot.lastOutcome());
    }
    @Test public void invalidActionInsideUnselectedBranchStillFailsDuringInit() {
        AutoFile file=AutoFile.parse(Json.parse("{\"formatVersion\":4,\"start\":{\"pose\":{\"xIn\":0,\"yIn\":0}},\"steps\":["
                +"{\"kind\":\"branch\",\"condition\":\"queueFull\",\"then\":[{\"kind\":\"command\",\"name\":\"intakeSet\",\"args\":{\"power\":2}}],"
                +"\"else\":[{\"kind\":\"wait\",\"seconds\":0.01}]}]}","badBranch"),"badBranch",Collections.emptyMap());
        assertFalse(robot.transfer.isFull());
        assertThrows(IllegalArgumentException.class,()->AutoBuilder.build(file,context));
    }
    @Test public void localizationFaultRemainsPrimaryWhenMotorCleanupAlsoFails() {
        Harness op=new Harness(); op.hardwareMap=map; robot.init(op);
        robot.intake=new IntakeSubsystem(map) {
            @Override public void stop() { throw new IllegalStateException("intake disconnected during stop"); }
        };
        IllegalStateException failure=assertThrows(IllegalStateException.class,()->robot.periodic(op));
        assertTrue(failure.getMessage().contains("Pinpoint localization"));
        assertEquals("intake disconnected during stop",failure.getSuppressed()[0].getMessage());
        assertEquals("FAULT",robot.lastOutcome());
        assertFalse(robot.gate.isOpenCommanded()); assertEquals(0,power("transfer_motor_2"),0);
    }
    @Test public void autonomousInitAndLocalizationFaultNeverHandAPoseToTeleOp() {
        MotionSample earlier = new MotionSample(System.nanoTime() * 1e-9, .5, .5, 0, 0, 0, 0, 0, .02, true, true, "earlier run");
        AutoPoseHandoff.save(TurretSubsystem.Alliance.RED, earlier);
        Harness op=new Harness(); op.hardwareMap=map; robot.init(op);
        assertFalse(AutoPoseHandoff.take(TurretSubsystem.Alliance.RED, System.nanoTime() * 1e-9).pending());
        AutoPoseHandoff.save(TurretSubsystem.Alliance.RED, earlier);
        assertThrows(IllegalStateException.class,()->robot.periodic(op)); // Pinpoint has no field reference.
        assertFalse(AutoPoseHandoff.take(TurretSubsystem.Alliance.RED, System.nanoTime() * 1e-9).pending());
    }
    @Test public void actualZenithUiSavedRoutineRunsThroughNativeFactoriesAndStops() throws Exception {
        AutoFile file=AutoFile.load(AutoSource.directory(new File("src/test/resources/zenith")),"new-auto");
        robot.observeSteps(file.steps);
        context.pose=file.startPose;
        AutoContext runtime=new AutoContext() {
            @Override public com.pedropathing.follower.Follower follower() { return context.follower; }
            @Override public com.pedropathing.paths.Path atSpeed(com.pedropathing.paths.Path path,double speed) { return context.atSpeed(path,speed); }
            @Override public Object drive() { return robot.drive; }
            @Override public Pose alliance(Pose p) { return p; }
            @Override public long nanoTime() { return now; }
            @Override public Command step(String id,Command body) { return robot.step(id,body); }
        };
        Command routine=AutoBuilder.build(file,runtime); Scheduler.schedule(routine);
        for (int i=0;i<500 && Scheduler.isScheduled(routine);i++) {
            now+=20_000_000; map.updateAll(.02); robot.turret.update(now*1e-9); robot.transfer.readSensors(now*1e-9);
            context.now=now; context.advance(); Scheduler.execute();
            robot.outputs.apply(now*1e-9,ControllerFixtures.pose(now*1e-9,1,-1,0,0,0,0).withReference(1),
                    robot.turret,robot.shots,robot.firing,robot.gate,robot.transfer,robot.intake);
        }
        assertFalse(Scheduler.isScheduled(routine));
        assertEquals(Collections.singletonList(.15),context.speeds);
        assertTrue(context.followed.get(0).getSegments().get(0).curve.getClass().getSimpleName().contains("Bezier"));
        assertEquals("TIMEOUT",robot.lastOutcome()); // False queueFull wait, preserved by stopAll.
        assertFalse(robot.gate.isOpenCommanded()); assertFalse(robot.transfer.isFeeding());
        assertEquals(0,power("intake_motor"),0); assertEquals(0,robot.turret.getTargetShooterRpm(),0);
    }
    private static final class Harness extends BioBuzzAutoFromFile {
        Harness() {
            spec=AutoFile.parse(Json.parse("{\"formatVersion\":4,\"alliance\":\"RED\",\"start\":{\"pose\":{\"xIn\":0,\"yIn\":0}},\"steps\":[{\"kind\":\"wait\",\"seconds\":0.1}]}","fixture"),"fixture",Collections.emptyMap());
            telemetry=(Telemetry)Proxy.newProxyInstance(getClass().getClassLoader(),new Class<?>[]{Telemetry.class},(p,m,a)->null);
        }
        @Override protected String autoName() { return "fixture"; }
        @Override protected ZenithRobot createRobot() { return BioBuzzRobot.pollenTest(); }
    }
}
