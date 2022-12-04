/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.internal;

import android.app.ActionBar;
import android.app.Activity;
import android.app.ActivityManager;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.IBinder;
import android.preference.PreferenceManager;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.StringRes;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.webkit.WebView;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.LinearLayout.LayoutParams;
import android.widget.PopupMenu;
import android.widget.TextView;

import com.google.blocks.ftcrobotcontroller.ProgrammingWebHandlers;
import com.google.blocks.ftcrobotcontroller.runtime.BlocksOpMode;
import com.qualcomm.ftccommon.ClassManagerFactory;
import com.qualcomm.ftccommon.FtcAboutActivity;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.ftccommon.FtcEventLoopIdle;
import com.qualcomm.ftccommon.FtcRobotControllerService;
import com.qualcomm.ftccommon.FtcRobotControllerService.FtcRobotControllerBinder;
import com.qualcomm.ftccommon.FtcRobotControllerSettingsActivity;
import com.qualcomm.ftccommon.LaunchActivityConstantsList;
import com.qualcomm.ftccommon.LaunchActivityConstantsList.RequestCode;
import com.qualcomm.ftccommon.Restarter;
import com.qualcomm.ftccommon.UpdateUI;
import com.qualcomm.ftccommon.configuration.EditParameters;
import com.qualcomm.ftccommon.configuration.FtcLoadFileActivity;
import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.ftcrobotcontroller.BuildConfig;
import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.util.ClockWarningSource;
import com.qualcomm.robotcore.util.Device;
import com.qualcomm.robotcore.util.Dimmer;
import com.qualcomm.robotcore.util.ImmersiveMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebServer;
import com.qualcomm.robotcore.wifi.NetworkConnection;
import com.qualcomm.robotcore.wifi.NetworkConnectionFactory;
import com.qualcomm.robotcore.wifi.NetworkType;

import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor;
import org.firstinspires.ftc.ftccommon.internal.AnnotatedHooksClassFilter;
import org.firstinspires.ftc.ftccommon.internal.FtcRobotControllerWatchdogService;
import org.firstinspires.ftc.ftccommon.internal.ProgramAndManageActivity;
import org.firstinspires.ftc.onbotjava.ExternalLibraries;
import org.firstinspires.ftc.onbotjava.OnBotJavaHelperImpl;
import org.firstinspires.ftc.onbotjava.OnBotJavaProgrammingMode;
import org.firstinspires.ftc.robotcore.external.navigation.MotionDetection;
import org.firstinspires.ftc.robotcore.internal.hardware.android.AndroidBoard;
import org.firstinspires.ftc.robotcore.internal.network.DeviceNameManagerFactory;
import org.firstinspires.ftc.robotcore.internal.network.PreferenceRemoterRC;
import org.firstinspires.ftc.robotcore.internal.network.StartResult;
import org.firstinspires.ftc.robotcore.internal.network.WifiDirectChannelChanger;
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteEvent;
import org.firstinspires.ftc.robotcore.internal.network.WifiMuteStateMachine;
import org.firstinspires.ftc.robotcore.internal.opmode.ClassManager;
import org.firstinspires.ftc.robotcore.internal.opmode.OnBotJavaHelper;
import org.firstinspires.ftc.robotcore.internal.system.AppAliveNotifier;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.robotcore.internal.system.PreferencesHelper;
import org.firstinspires.ftc.robotcore.internal.system.ServiceController;
import org.firstinspires.ftc.robotcore.internal.ui.ThemedActivity;
import org.firstinspires.ftc.robotcore.internal.ui.UILocation;
import org.firstinspires.ftc.robotcore.internal.webserver.RobotControllerWebInfo;
import org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingModeManager;
import org.firstinspires.inspection.RcInspectionActivity;
import org.threeten.bp.YearMonth;
import org.xmlpull.v1.XmlPullParserException;

import java.io.FileNotFoundException;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

@SuppressWarnings("WeakerAccess")
public class FtcRobotControllerActivity extends Activity
  {
  public static final String TAG = "RCActivity";
  public String getTag() { return TAG; }

  private static final int REQUEST_CONFIG_WIFI_CHANNEL = 1;
  private static final int NUM_GAMEPADS = 2;

  protected WifiManager.WifiLock wifiLock;
  protected RobotConfigFileManager cfgFileMgr;

  private OnBotJavaHelper onBotJavaHelper;

  protected ProgrammingModeManager programmingModeManager;

  protected UpdateUI.Callback callback;
  protected Context context;
  protected Utility utility;
  protected StartResult prefRemoterStartResult = new StartResult();
  protected StartResult deviceNameStartResult = new StartResult();
  protected PreferencesHelper preferencesHelper;
  protected final SharedPreferencesListener sharedPreferencesListener = new SharedPreferencesListener();

  protected ImageButton buttonMenu;
  protected TextView textDeviceName;
  protected TextView textNetworkConnectionStatus;
  protected TextView textRobotStatus;
  protected TextView[] textGamepad = new TextView[NUM_GAMEPADS];
  protected TextView textOpMode;
  protected TextView textErrorMessage;
  protected ImmersiveMode immersion;

  protected UpdateUI updateUI;
  protected Dimmer dimmer;
  protected LinearLayout entireScreenLayout;

  protected FtcRobotControllerService controllerService;
  protected NetworkType networkType;

  protected FtcEventLoop eventLoop;
  protected Queue<UsbDevice> receivedUsbAttachmentNotifications;

  protected WifiMuteStateMachine wifiMuteStateMachine;
  protected MotionDetection motionDetection;

  private static boolean permissionsValidated = false;

  private WifiDirectChannelChanger wifiDirectChannelChanger;

  protected class RobotRestarter implements Restarter {

    public void requestRestart() {
      requestRobotRestart();
    }

  }

  protected boolean serviceShouldUnbind = false;
  protected ServiceConnection connection = new ServiceConnection() {
    @Override
    public void onServiceConnected(ComponentName name, IBinder service) {
      FtcRobotControllerBinder binder = (FtcRobotControllerBinder) service;
      onServiceBind(binder.getService());
    }

    @Override
    public void onServiceDisconnected(ComponentName name) {
      RobotLog.vv(FtcRobotControllerService.TAG, "%s.controllerService=null", TAG);
      controllerService = null;
    }
  };

  @Override
  protected void onNewIntent(Intent intent) {
    super.onNewIntent(intent);

    if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(intent.getAction())) {
      UsbDevice usbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
      RobotLog.vv(TAG, "ACTION_USB_DEVICE_ATTACHED: %s", usbDevice.getDeviceName());

      if (usbDevice != null) {  // paranoia
        // We might get attachment notifications before the event loop is set up, so
        // we hold on to them and pass them along only when we're good and ready.
        if (receivedUsbAttachmentNotifications != null) { // *total* paranoia
          receivedUsbAttachmentNotifications.add(usbDevice);
          passReceivedUsbAttachmentsToEventLoop();
        }
      }
    }
  }

  protected void passReceivedUsbAttachmentsToEventLoop() {
    if (this.eventLoop != null) {
      for (;;) {
        UsbDevice usbDevice = receivedUsbAttachmentNotifications.poll();
        if (usbDevice == null)
          break;
        this.eventLoop.onUsbDeviceAttached(usbDevice);
      }
    }
    else {
      // Paranoia: we don't want the pending list to grow without bound when we don't
      // (yet) have an event loop
      while (receivedUsbAttachmentNotifications.size() > 100) {
        receivedUsbAttachmentNotifications.poll();
      }
    }
  }

  /**
   * There are cases where a permission may be revoked and the system restart will restart the
   * FtcRobotControllerActivity, instead of the launch activity.  Detect when that happens, and throw
   * the device back to the permission validator activity.
   */
  protected boolean enforcePermissionValidator() {
    if (!permissionsValidated) {
      RobotLog.vv(TAG, "Redirecting to permission validator");
      Intent permissionValidatorIntent = new Intent(AppUtil.getDefContext(), PermissionValidatorWrapper.class);
      startActivity(permissionValidatorIntent);
      finish();
      return true;
    } else {
      RobotLog.vv(TAG, "Permissions validated already");
      return false;
    }
  }

  public static void setPermissionsValidated() {
    permissionsValidated = true;
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    if (enforcePermissionValidator()) {
      return;
    }

    RobotLog.onApplicationStart();  // robustify against onCreate() following onDestroy() but using the same app instance, which apparently does happen
    RobotLog.vv(TAG, "onCreate()");
    ThemedActivity.appAppThemeToActivity(getTag(), this); // do this way instead of inherit to help AppInventor

    // Oddly, sometimes after a crash & restart the root activity will be something unexpected, like from the before crash? We don't yet understand
    RobotLog.vv(TAG, "rootActivity is of class %s", AppUtil.getInstance().getRootActivity().getClass().getSimpleName());
    RobotLog.vv(TAG, "launchActivity is of class %s", FtcRobotControllerWatchdogService.launchActivity());
    Assert.assertTrue(FtcRobotControllerWatchdogService.isLaunchActivity(AppUtil.getInstance().getRootActivity()));
    Assert.assertTrue(AppUtil.getInstance().isRobotController());

    // Quick check: should we pretend we're not here, and so allow the Lynx to operate as
    // a stand-alone USB-connected module?
    if (LynxConstants.isRevControlHub()) {
      // Double-sure check that we can talk to the DB over the serial TTY
      AndroidBoard.getInstance().getAndroidBoardIsPresentPin().setState(true);
    }

    context = this;
    utility = new Utility(this);

    DeviceNameManagerFactory.getInstance().start(deviceNameStartResult);

    PreferenceRemoterRC.getInstance().start(prefRemoterStartResult);

    receivedUsbAttachmentNotifications = new ConcurrentLinkedQueue<UsbDevice>();
    eventLoop = null;

    setContentView(R.layout.activity_ftc_controller);

    preferencesHelper = new PreferencesHelper(TAG, context);
    preferencesHelper.writeBooleanPrefIfDifferent(context.getString(R.string.pref_rc_connected), true);
    preferencesHelper.getSharedPreferences().registerOnSharedPreferenceChangeListener(sharedPreferencesListener);

    // Check if this RC app is from a later FTC season than what was installed previously
    int ftcSeasonYearOfPreviouslyInstalledRc = preferencesHelper.readInt(getString(R.string.pref_ftc_season_year_of_current_rc), 0);
    int ftcSeasonYearOfCurrentlyInstalledRc = AppUtil.getInstance().getFtcSeasonYear(AppUtil.getInstance().getLocalSdkBuildMonth()).getValue();
    if (ftcSeasonYearOfCurrentlyInstalledRc > ftcSeasonYearOfPreviouslyInstalledRc) {
      preferencesHelper.writeIntPrefIfDifferent(getString(R.string.pref_ftc_season_year_of_current_rc), ftcSeasonYearOfCurrentlyInstalledRc);
      // Since it's a new FTC season, we should reset certain settings back to their default values.
      preferencesHelper.writeBooleanPrefIfDifferent(getString(R.string.pref_warn_about_2_4_ghz_band), true);
      preferencesHelper.writeBooleanPrefIfDifferent(getString(R.string.pref_warn_about_obsolete_software), true);
      preferencesHelper.writeBooleanPrefIfDifferent(getString(R.string.pref_warn_about_mismatched_app_versions), true);
      preferencesHelper.writeBooleanPrefIfDifferent(getString(R.string.pref_warn_about_incorrect_clocks), true);
    }

    entireScreenLayout = (LinearLayout) findViewById(R.id.entire_screen);
    buttonMenu = (ImageButton) findViewById(R.id.menu_buttons);
    buttonMenu.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View v) {
        PopupMenu popupMenu = new PopupMenu(FtcRobotControllerActivity.this, v);
        popupMenu.setOnMenuItemClickListener(new PopupMenu.OnMenuItemClickListener() {
          @Override
          public boolean onMenuItemClick(MenuItem item) {
            return onOptionsItemSelected(item); // Delegate to the handler for the hardware menu button
          }
        });
        popupMenu.inflate(R.menu.ftc_robot_controller);
        AnnotatedHooksClassFilter.getInstance().callOnCreateMenuMethods(
            FtcRobotControllerActivity.this, popupMenu.getMenu());
        popupMenu.show();
      }
    });

    updateMonitorLayout(getResources().getConfiguration());

    BlocksOpMode.setActivityAndWebView(this, (WebView) findViewById(R.id.webViewBlocksRuntime));

    ExternalLibraries.getInstance().onCreate();
    onBotJavaHelper = new OnBotJavaHelperImpl();

    /*
     * Paranoia as the ClassManagerFactory requires EXTERNAL_STORAGE permissions
     * and we've seen on the DS where the finish() call above does not short-circuit
     * the onCreate() call for the activity and then we crash here because we don't
     * have permissions. So...
     */
    if (permissionsValidated) {
      ClassManager.getInstance().setOnBotJavaClassHelper(onBotJavaHelper);
      ClassManagerFactory.registerFilters();
      ClassManagerFactory.processAllClasses();
    }

    cfgFileMgr = new RobotConfigFileManager(this);

    // Clean up 'dirty' status after a possible crash
    RobotConfigFile configFile = cfgFileMgr.getActiveConfig();
    if (configFile.isDirty()) {
      configFile.markClean();
      cfgFileMgr.setActiveConfig(false, configFile);
    }

    textDeviceName = (TextView) findViewById(R.id.textDeviceName);
    textNetworkConnectionStatus = (TextView) findViewById(R.id.textNetworkConnectionStatus);
    textRobotStatus = (TextView) findViewById(R.id.textRobotStatus);
    textOpMode = (TextView) findViewById(R.id.textOpMode);
    textErrorMessage = (TextView) findViewById(R.id.textErrorMessage);
    textGamepad[0] = (TextView) findViewById(R.id.textGamepad1);
    textGamepad[1] = (TextView) findViewById(R.id.textGamepad2);
    immersion = new ImmersiveMode(getWindow().getDecorView());
    dimmer = new Dimmer(this);
    dimmer.longBright();

    programmingModeManager = new ProgrammingModeManager();
    programmingModeManager.register(new ProgrammingWebHandlers());
    programmingModeManager.register(new OnBotJavaProgrammingMode());

    updateUI = createUpdateUI();
    callback = createUICallback(updateUI);

    PreferenceManager.setDefaultValues(this, R.xml.app_settings, false);

    WifiManager wifiManager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
    wifiLock = wifiManager.createWifiLock(WifiManager.WIFI_MODE_FULL_HIGH_PERF, "");

    hittingMenuButtonBrightensScreen();

    wifiLock.acquire();
    callback.networkConnectionUpdate(NetworkConnection.NetworkEvent.DISCONNECTED);
    readNetworkType();
    ServiceController.startService(FtcRobotControllerWatchdogService.class);
    bindToService();
    RobotLog.logAppInfo();
    RobotLog.logDeviceInfo();
    AndroidBoard.getInstance().logAndroidBoardInfo();

    if (preferencesHelper.readBoolean(getString(R.string.pref_wifi_automute), false)) {
      initWifiMute(true);
    }

    FtcAboutActivity.setBuildTimeFromBuildConfig(BuildConfig.APP_BUILD_TIME);

    // check to see if there is a preferred Wi-Fi to use.
    checkPreferredChannel();

    AnnotatedHooksClassFilter.getInstance().callOnCreateMethods(this);
  }

  protected UpdateUI createUpdateUI() {
    Restarter restarter = new RobotRestarter();
    UpdateUI result = new UpdateUI(this, dimmer);
    result.setRestarter(restarter);
    result.setTextViews(textNetworkConnectionStatus, textRobotStatus, textGamepad, textOpMode, textErrorMessage, textDeviceName);
    return result;
  }

  protected UpdateUI.Callback createUICallback(UpdateUI updateUI) {
    UpdateUI.Callback result = updateUI.new Callback();
    result.setStateMonitor(new SoundPlayingRobotMonitor());
    return result;
  }

  @Override
  protected void onStart() {
    super.onStart();
    RobotLog.vv(TAG, "onStart()");

    entireScreenLayout.setOnTouchListener(new View.OnTouchListener() {
      @Override
      public boolean onTouch(View v, MotionEvent event) {
        dimmer.handleDimTimer();
        return false;
      }
    });
  }

  @Override
  protected void onResume() {
    super.onResume();
    RobotLog.vv(TAG, "onResume()");

    // In case the user just got back from fixing their clock, refresh ClockWarningSource
    ClockWarningSource.getInstance().onPossibleRcClockUpdate();
  }

  @Override
  protected void onPause() {
    super.onPause();
    RobotLog.vv(TAG, "onPause()");
  }

  @Override
  protected void onStop() {
    // Note: this gets called even when the configuration editor is launched. That is, it gets
    // called surprisingly often. So, we don't actually do much here.
    super.onStop();
    RobotLog.vv(TAG, "onStop()");
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
    RobotLog.vv(TAG, "onDestroy()");

    shutdownRobot();  // Ensure the robot is put away to bed
    if (callback != null) callback.close();

    PreferenceRemoterRC.getInstance().stop(prefRemoterStartResult);
    DeviceNameManagerFactory.getInstance().stop(deviceNameStartResult);

    unbindFromService();
    // If the app manually (?) is stopped, then we don't need the auto-starting function (?)
    ServiceController.stopService(FtcRobotControllerWatchdogService.class);
    if (wifiLock != null) wifiLock.release();
    if (preferencesHelper != null) preferencesHelper.getSharedPreferences().unregisterOnSharedPreferenceChangeListener(sharedPreferencesListener);

    RobotLog.cancelWriteLogcatToDisk();

    AnnotatedHooksClassFilter.getInstance().callOnDestroyMethods(this);
  }

  protected void bindToService() {
    readNetworkType();
    Intent intent = new Intent(this, FtcRobotControllerService.class);
    intent.putExtra(NetworkConnectionFactory.NETWORK_CONNECTION_TYPE, networkType);
    serviceShouldUnbind = bindService(intent, connection, Context.BIND_AUTO_CREATE);
  }

  protected void unbindFromService() {
    if (serviceShouldUnbind) {
      unbindService(connection);
      serviceShouldUnbind = false;
    }
  }

  protected void readNetworkType() {
    // Control hubs are always running the access point model.  Everything else, for the time
    // being always runs the Wi-Fi Direct model.
    if (Device.isRevControlHub() == true) {
      networkType = NetworkType.RCWIRELESSAP;
    } else {
      networkType = NetworkType.fromString(preferencesHelper.readString(context.getString(R.string.pref_pairing_kind), NetworkType.globalDefaultAsString()));
    }

    // update the app_settings
    preferencesHelper.writeStringPrefIfDifferent(context.getString(R.string.pref_pairing_kind), networkType.toString());
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus) {
    super.onWindowFocusChanged(hasFocus);

    if (hasFocus) {
      immersion.hideSystemUI();
      getWindow().setFlags(WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION, WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION);
    }
  }

  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    getMenuInflater().inflate(R.menu.ftc_robot_controller, menu);
    AnnotatedHooksClassFilter.getInstance().callOnCreateMenuMethods(this, menu);
    return true;
  }

  private boolean isRobotRunning() {
    if (controllerService == null) {
      return false;
    }

    Robot robot = controllerService.getRobot();

    if ((robot == null) || (robot.eventLoopManager == null)) {
      return false;
    }

    RobotState robotState = robot.eventLoopManager.state;

    if (robotState != RobotState.RUNNING) {
      return false;
    } else {
      return true;
    }
  }

  @Override
  public boolean onOptionsItemSelected(MenuItem item) {
    int id = item.getItemId();

    if (id == R.id.action_program_and_manage) {
      if (isRobotRunning()) {
        Intent programmingModeIntent = new Intent(AppUtil.getDefContext(), ProgramAndManageActivity.class);
        RobotControllerWebInfo webInfo = programmingModeManager.getWebServer().getConnectionInformation();
        programmingModeIntent.putExtra(LaunchActivityConstantsList.RC_WEB_INFO, webInfo.toJson());
        startActivity(programmingModeIntent);
      } else {
        AppUtil.getInstance().showToast(UILocation.ONLY_LOCAL, context.getString(R.string.toastWifiUpBeforeProgrammingMode));
      }
    } else if (id == R.id.action_inspection_mode) {
      Intent inspectionModeIntent = new Intent(AppUtil.getDefContext(), RcInspectionActivity.class);
      startActivity(inspectionModeIntent);
      return true;
    } else if (id == R.id.action_restart_robot) {
      dimmer.handleDimTimer();
      AppUtil.getInstance().showToast(UILocation.BOTH, context.getString(R.string.toastRestartingRobot));
      requestRobotRestart();
      return true;
    }
    else if (id == R.id.action_configure_robot) {
      EditParameters parameters = new EditParameters();
      Intent intentConfigure = new Intent(AppUtil.getDefContext(), FtcLoadFileActivity.class);
      parameters.putIntent(intentConfigure);
      startActivityForResult(intentConfigure, RequestCode.CONFIGURE_ROBOT_CONTROLLER.ordinal());
    }
    else if (id == R.id.action_settings) {
	  // historical: this once erroneously used FTC_CONFIGURE_REQUEST_CODE_ROBOT_CONTROLLER
      Intent settingsIntent = new Intent(AppUtil.getDefContext(), FtcRobotControllerSettingsActivity.class);
      startActivityForResult(settingsIntent, RequestCode.SETTINGS_ROBOT_CONTROLLER.ordinal());
      return true;
    }
    else if (id == R.id.action_about) {
      Intent intent = new Intent(AppUtil.getDefContext(), FtcAboutActivity.class);
      startActivity(intent);
      return true;
    }
    else if (id == R.id.action_exit_app) {

      //Clear backstack and everything to prevent edge case where VM might be
      //restarted (after it was exited) if more than one activity was on the
      //backstack for some reason.
      finishAffinity();

      //For lollipop and up, we can clear ourselves from the recents list too
      if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.LOLLIPOP) {
        ActivityManager manager = (ActivityManager) getSystemService(ACTIVITY_SERVICE);
        List<ActivityManager.AppTask> tasks = manager.getAppTasks();

        for (ActivityManager.AppTask task : tasks) {
          task.finishAndRemoveTask();
        }
      }

      // Allow the user to use the Control Hub operating system's UI, instead of relaunching the app
      AppAliveNotifier.getInstance().disableAppWatchdogUntilNextAppStart();

      //Finally, nuke the VM from orbit
      AppUtil.getInstance().exitApplication();

      return true;
    }

   return super.onOptionsItemSelected(item);
  }

  @Override
  public void onConfigurationChanged(Configuration newConfig) {
    super.onConfigurationChanged(newConfig);
    // don't destroy assets on screen rotation
    updateMonitorLayout(newConfig);
  }

  /**
   * Updates the orientation of monitorContainer (which contains cameraMonitorView and
   * tfodMonitorView) based on the given configuration. Makes the children split the space.
   */
  private void updateMonitorLayout(Configuration configuration) {
    LinearLayout monitorContainer = (LinearLayout) findViewById(R.id.monitorContainer);
    if (configuration.orientation == Configuration.ORIENTATION_LANDSCAPE) {
      // When the phone is landscape, lay out the monitor views horizontally.
      monitorContainer.setOrientation(LinearLayout.HORIZONTAL);
      for (int i = 0; i < monitorContainer.getChildCount(); i++) {
        View view = monitorContainer.getChildAt(i);
        view.setLayoutParams(new LayoutParams(0, LayoutParams.MATCH_PARENT, 1 /* weight */));
      }
    } else {
      // When the phone is portrait, lay out the monitor views vertically.
      monitorContainer.setOrientation(LinearLayout.VERTICAL);
      for (int i = 0; i < monitorContainer.getChildCount(); i++) {
        View view = monitorContainer.getChildAt(i);
        view.setLayoutParams(new LayoutParams(LayoutParams.MATCH_PARENT, 0, 1 /* weight */));
      }
    }
    monitorContainer.requestLayout();
  }

  @Override
  protected void onActivityResult(int request, int result, Intent intent) {
    if (request == REQUEST_CONFIG_WIFI_CHANNEL) {
      if (result == RESULT_OK) {
        AppUtil.getInstance().showToast(UILocation.BOTH, context.getString(R.string.toastWifiConfigurationComplete));
      }
    }
    // was some historical confusion about launch codes here, so we err safely
    if (request == RequestCode.CONFIGURE_ROBOT_CONTROLLER.ordinal() || request == RequestCode.SETTINGS_ROBOT_CONTROLLER.ordinal()) {
      // We always do a refresh, whether it was a cancel or an OK, for robustness
      shutdownRobot();
      cfgFileMgr.getActiveConfigAndUpdateUI();
      updateUIAndRequestRobotSetup();
    }
  }

  public void onServiceBind(final FtcRobotControllerService service) {
    RobotLog.vv(FtcRobotControllerService.TAG, "%s.controllerService=bound", TAG);
    controllerService = service;
    updateUI.setControllerService(controllerService);

    controllerService.setOnBotJavaHelper(onBotJavaHelper);

    updateUIAndRequestRobotSetup();
    programmingModeManager.setState(new FtcRobotControllerServiceState() {
      @NonNull
      @Override
      public WebServer getWebServer() {
        return service.getWebServer();
      }

      @Nullable
      @Override
      public OnBotJavaHelper getOnBotJavaHelper() {
        return service.getOnBotJavaHelper();
      }

      @Override
      public EventLoopManager getEventLoopManager() {
        return service.getRobot().eventLoopManager;
      }
    });

    AnnotatedHooksClassFilter.getInstance().callWebHandlerRegistrarMethods(this,
        service.getWebServer().getWebHandlerManager());
  }

  private void updateUIAndRequestRobotSetup() {
    if (controllerService != null) {
      callback.networkConnectionUpdate(controllerService.getNetworkConnectionStatus());
      callback.updateRobotStatus(controllerService.getRobotStatus());
      // Only show this first-time toast on headless systems: what we have now on non-headless suffices
      requestRobotSetup(LynxConstants.isRevControlHub()
        ? new Runnable() {
            @Override public void run() {
              showRestartRobotCompleteToast(R.string.toastRobotSetupComplete);
            }
          }
        : null);
    }
  }

  private void requestRobotSetup(@Nullable Runnable runOnComplete) {
    if (controllerService == null) return;

    RobotConfigFile file = cfgFileMgr.getActiveConfigAndUpdateUI();
    HardwareFactory hardwareFactory = new HardwareFactory(context);
    try {
      hardwareFactory.setXmlPullParser(file.getXml());
    } catch (FileNotFoundException | XmlPullParserException e) {
      RobotLog.ww(TAG, e, "Unable to set configuration file %s. Falling back on noConfig.", file.getName());
      file = RobotConfigFile.noConfig(cfgFileMgr);
      try {
        hardwareFactory.setXmlPullParser(file.getXml());
        cfgFileMgr.setActiveConfigAndUpdateUI(false, file);
      } catch (FileNotFoundException | XmlPullParserException e1) {
        RobotLog.ee(TAG, e1, "Failed to fall back on noConfig");
      }
    }

    OpModeRegister userOpModeRegister = createOpModeRegister();
    eventLoop = new FtcEventLoop(hardwareFactory, userOpModeRegister, callback, this);
    FtcEventLoopIdle idleLoop = new FtcEventLoopIdle(hardwareFactory, userOpModeRegister, callback, this);

    controllerService.setCallback(callback);
    controllerService.setupRobot(eventLoop, idleLoop, runOnComplete);

    passReceivedUsbAttachmentsToEventLoop();
    AndroidBoard.showErrorIfUnknownControlHub();

    AnnotatedHooksClassFilter.getInstance().callOnCreateEventLoopMethods(this, eventLoop);
  }

  protected OpModeRegister createOpModeRegister() {
    return new FtcOpModeRegister();
  }

  private void shutdownRobot() {
    if (controllerService != null) controllerService.shutdownRobot();
  }

  private void requestRobotRestart() {
    AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getDefContext().getString(R.string.toastRestartingRobot));
    //
    RobotLog.clearGlobalErrorMsg();
    RobotLog.clearGlobalWarningMsg();
    shutdownRobot();
    requestRobotSetup(new Runnable() {
      @Override public void run() {
        showRestartRobotCompleteToast(R.string.toastRestartRobotComplete);
        }
      });
  }

  private void showRestartRobotCompleteToast(@StringRes int resid) {
    AppUtil.getInstance().showToast(UILocation.BOTH, AppUtil.getDefContext().getString(resid));
  }

  private void checkPreferredChannel() {
    // For P2P network, check to see what preferred channel is.
    if (networkType ==  NetworkType.WIFIDIRECT) {
      int prefChannel = preferencesHelper.readInt(getString(com.qualcomm.ftccommon.R.string.pref_wifip2p_channel), -1);
      if (prefChannel == -1) {
        prefChannel = 0;
        RobotLog.vv(TAG, "pref_wifip2p_channel: No preferred channel defined. Will use a default value of %d", prefChannel);
      } else {
        RobotLog.vv(TAG, "pref_wifip2p_channel: Found existing preferred channel (%d).", prefChannel);
      }

      // attempt to set the preferred channel.
      RobotLog.vv(TAG, "pref_wifip2p_channel: attempting to set preferred channel...");
      wifiDirectChannelChanger = new WifiDirectChannelChanger();
      wifiDirectChannelChanger.changeToChannel(prefChannel);
    }
  }

  protected void hittingMenuButtonBrightensScreen() {
    ActionBar actionBar = getActionBar();
    if (actionBar != null) {
      actionBar.addOnMenuVisibilityListener(new ActionBar.OnMenuVisibilityListener() {
        @Override
        public void onMenuVisibilityChanged(boolean isVisible) {
          if (isVisible) {
            dimmer.handleDimTimer();
          }
        }
      });
    }
  }

  protected class SharedPreferencesListener implements SharedPreferences.OnSharedPreferenceChangeListener {
    @Override public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
      if (key.equals(context.getString(R.string.pref_app_theme))) {
        ThemedActivity.restartForAppThemeChange(getTag(), getString(R.string.appThemeChangeRestartNotifyRC));
      } else if (key.equals(context.getString(R.string.pref_wifi_automute))) {
        if (preferencesHelper.readBoolean(context.getString(R.string.pref_wifi_automute), false)) {
          initWifiMute(true);
        } else {
          initWifiMute(false);
        }
      }
    }
  }

  protected void initWifiMute(boolean enable) {
    if (enable) {
      wifiMuteStateMachine = new WifiMuteStateMachine();
      wifiMuteStateMachine.initialize();
      wifiMuteStateMachine.start();

      motionDetection = new MotionDetection(2.0, 10);
      motionDetection.startListening();
      motionDetection.registerListener(new MotionDetection.MotionDetectionListener() {
        @Override
        public void onMotionDetected(double vector)
        {
          wifiMuteStateMachine.consumeEvent(WifiMuteEvent.USER_ACTIVITY);
        }
      });
    } else {
      wifiMuteStateMachine.stop();
      wifiMuteStateMachine = null;
      motionDetection.stopListening();
      motionDetection.purgeListeners();
      motionDetection = null;
    }
  }

  @Override
  public void onUserInteraction() {
    if (wifiMuteStateMachine != null) {
      wifiMuteStateMachine.consumeEvent(WifiMuteEvent.USER_ACTIVITY);
    }
  }
}
