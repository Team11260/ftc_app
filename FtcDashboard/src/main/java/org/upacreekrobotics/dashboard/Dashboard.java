package org.upacreekrobotics.dashboard;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.BatteryChecker;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.upacreekrobotics.eventloop.OurEventLoop;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class Dashboard implements OpModeManagerImpl.Notifications, BatteryChecker.BatteryWatcher {
    public static final String TAG = "RobotDashboard";
    private Thread dataThread;
    private Thread dashboardThread;
    private Data data;
    private EventLoop eventLoop;
    private RobotStatus status;
    private dashboardtelemetry DashboardTelemtry;
    private smartdashboard SmartDashboard;
    private String lastInputValue;
    private boolean connected = false;
    private List<VoltageSensor> voltageSensors = null;
    private boolean restartRequested = false;
    private boolean isRunning = false;
    private BatteryChecker.BatteryWatcher batteryWatcher;
    private BatteryChecker batteryChecker;
    private Date date;
    private String oldTelemetry = "";
    private String infoText = "";
    private long opModeInitTime = 0;
    private long opModeStartTime = 0;
    private String activeOpModeName = "";
    private String oldSmartdashboard = "";
    private int smartdashboardRequestID = 0;
    private Object smartdashboardRequestIDLock = new Object();
    private List<String> smartdashboardResponses = Collections.synchronizedList(new ArrayList<>());

    private OpModeManagerImpl opModeManager;
    private RobotStatus.OpModeStatus activeOpModeStatus = RobotStatus.OpModeStatus.STOPPED;
    private RobotStatus.OpModeStatus requestedOpModeStatus = RobotStatus.OpModeStatus.STOPPED;
    private List<String> opModeList;
    private HashMap<String, HashMap<String, Field>> variables;

    private Context context;
    private SharedPreferences sharedPreferences;

    private static Dashboard dashboard;

    ////////////////Gradle Configuration////////////////
    private static final Set<String> IGNORED_PACKAGES = new HashSet<>(Arrays.asList(
            "java",
            "android",
            "com.sun",
            "com.vuforia",
            "com.google",
            "kotlin"
    ));

    ////////////////Start the dashboard, calls Dashboard constructor, called by "onCreate()" in "FtcRobotControllerActivity"////////////////
    public static void start() {

        if (dashboard == null) {
            dashboard = new Dashboard();
        }
    }

    ////////////////Passes the eventLoop, and android context to allow for opModeManager access////////////////
    ////////////////and phone battery level polling////////////////
    ////////////////called by "requestRobotSetup()" in "FtcRobotControllerActivity"////////////////
    public static void attachEventLoop(EventLoop eventLoop, Context ct) {
        dashboard.internalAttachEventLoop(eventLoop, ct);
    }

    ////////////////Stops the dashboard and socket connection////////////////
    ////////////////called by "onDestroy()" in "FtcRobotControllerActivity"////////////////
    public static void stop() {
        if (dashboard != null) {
            dashboard.isRunning = false;
            dashboard = null;
        }
    }

    ////////////////Returns current dashboard instance for use in user OpModes////////////////
    public static Dashboard getInstance() {
        return dashboard;
    }

    ////////////////Constructor called by "start()"////////////////
    private Dashboard() {

        isRunning = true;

        variables = new HashMap<>();

        ClasspathScanner scanner = new ClasspathScanner(new ClassFilter() {
            @Override
            public boolean shouldProcessClass(String className) {
                for (String packageName : IGNORED_PACKAGES) {
                    if (className.startsWith(packageName)) {
                        return false;
                    }
                }
                return true;
            }

            @Override
            public void processClass(Class klass) {
                if (klass.isAnnotationPresent(Config.class) && !klass.isAnnotationPresent(Disabled.class)) {

                    HashMap<String, Field> klassVariables = new HashMap<>();

                    for(Field field:klass.getDeclaredFields()) {
                        try {
                            if(Modifier.isStatic(field.getModifiers()) && Modifier.isPublic(field.getModifiers()) && !Modifier.isFinal(field.getModifiers()) && (field.get(null) instanceof  Integer || field.get(null) instanceof  Double || field.get(null) instanceof String)) {
                                klassVariables.put(field.getName(), field);
                            }
                        } catch (IllegalAccessException e) {
                            e.printStackTrace();
                        }
                    }

                    if(klassVariables.size() > 0) variables.put(klass.getSimpleName(), klassVariables);
                }
            }
        });
        scanner.scanClasspath();

        date = new Date();

        opModeList = Collections.synchronizedList(new ArrayList<>());

        dataThread = new Thread(new DataHandler());
        dataThread.start();
        dashboardThread = new Thread(new DashboardHandler());
        dashboardThread.start();
    }

    ////////////////Returns the current "RobotStatus", this may be useful in the future////////////////
    private void internalAttachEventLoop(EventLoop eventLoop, Context context) {

        this.context = context;
        sharedPreferences = context.getSharedPreferences("UP_A_CREEK_FTC_PREFERENCES", Context.MODE_PRIVATE);

        opModeManager = eventLoop.getOpModeManager();

        this.eventLoop = eventLoop;
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        synchronized (opModeList) {
            opModeList.clear();
        }

        (new Thread() {
            @Override
            public void run() {
                RegisteredOpModes.getInstance().waitOpModesRegistered();
                synchronized (opModeList) {
                    for (OpModeMeta opModeMeta : RegisteredOpModes.getInstance().getOpModes()) {
                        opModeList.add(opModeMeta.name);
                    }
                }
            }
        }).start();

        batteryWatcher = this;
        batteryChecker = new BatteryChecker(batteryWatcher, 60000);
    }

    ////////////////Returns the current "RobotStatus", this may be useful in the future////////////////
    private RobotStatus getRobotStatus() {
        if (opModeManager == null) {
            return new RobotStatus();
        } else {
            return new RobotStatus(opModeManager.getActiveOpModeName(), activeOpModeStatus);
        }
    }

    ////////////////This is the main loop, called by the "dashboardThread"////////////////
    synchronized void receiveMessage() {
        if (data != null) {
            ////////////////Receiving Message////////////////
            Message message = data.read();
            if (message != null) {
                switch (message.getType()) {
                    case GET_OP_MODES: {
                        for (String mode : opModeList) {
                            data.write(new Message(MessageType.OP_MODES, mode));
                        }
                        try {
                            if (!opModeManager.getActiveOpModeName().equals("$Stop$Robot$")) {
                                if (activeOpModeStatus.equals(RobotStatus.OpModeStatus.INIT)) {
                                    data.write(new Message(MessageType.SELECT_OP_MODE, opModeManager.getActiveOpModeName()));
                                    data.write(new Message(MessageType.ROBOT_STATUS, "INIT"));
                                } else if (dashboard.activeOpModeStatus.equals(RobotStatus.OpModeStatus.RUNNING)) {
                                    data.write(new Message(MessageType.SELECT_OP_MODE, opModeManager.getActiveOpModeName()));
                                    data.write(new Message(MessageType.ROBOT_STATUS, "RUNNING"));
                                }
                            }
                        } catch (NullPointerException e) {

                        }

                        for(HashMap.Entry<String, HashMap<String, Field>> entry:variables.entrySet()) {
                            String dataLine = entry.getKey();
                            for(HashMap.Entry<String, Field> e:entry.getValue().entrySet()) {
                                try {
                                    dataLine += "<&#%#&>" + e.getKey() + ">#&%&#<" + e.getValue().getType() + ">#&%&#<" + e.getValue().get(null);
                                } catch (IllegalAccessException e1) {
                                    e1.printStackTrace();
                                }
                            }
                            data.write(new Message(MessageType.VARIABLE, dataLine));
                        }

                        String[] telemetryParts = oldTelemetry.split("&#%#&");
                        for (String part : telemetryParts) {
                            if (!part.equals(" "))
                                data.write(new Message(MessageType.TELEMETRY, part));
                        }
                        oldTelemetry = "";

                        String[] smartdashboardParts = oldSmartdashboard.split("&#%#&");
                        for (String part : smartdashboardParts) {
                            if (!part.equals(" "))
                                data.write(new Message(MessageType.SMARTDASHBOARD_PUT, part));
                        }
                        oldSmartdashboard = "";

                        if (batteryChecker != null) batteryChecker.pollBatteryLevel(batteryWatcher);
                        break;
                    }

                    case VARIABLE: {
                        String[] parts = message.getText().split("<&#%#&>");
                        try {
                            if (variables.get(parts[0]).get(parts[1].split(">#&%&#<")[0]).get(null) instanceof Integer) variables.get(parts[0]).get(parts[1].split(">#&%&#<")[0]).set(null, Integer.valueOf(parts[1].split(">#&%&#<")[1]));
                            else if (variables.get(parts[0]).get(parts[1].split(">#&%&#<")[0]).get(null) instanceof Double) variables.get(parts[0]).get(parts[1].split(">#&%&#<")[0]).set(null, Double.valueOf(parts[1].split(">#&%&#<")[1]));
                            else variables.get(parts[0]).get(parts[1].split(">#&%&#<")[0]).set(null, Integer.valueOf(parts[1].split(">#&%&#<")[1]));
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                        break;
                    }

                    case SELECT_OP_MODE: {
                        status = new RobotStatus(message.getText(), activeOpModeStatus);
                        break;
                    }

                    case RESTART_ROBOT: {
                        restartRequested = true;
                        break;
                    }

                    case INIT_OP_MODE: {
                        activeOpModeStatus = RobotStatus.OpModeStatus.INIT;
                        requestedOpModeStatus = RobotStatus.OpModeStatus.INIT;
                        opModeManager.initActiveOpMode(status.getActiveOpMode());
                        break;
                    }

                    case RUN_OP_MODE: {
                        activeOpModeStatus = RobotStatus.OpModeStatus.RUNNING;
                        requestedOpModeStatus = RobotStatus.OpModeStatus.RUNNING;
                        opModeManager.startActiveOpMode();
                        break;
                    }

                    case STOP_OP_MODE: {
                        activeOpModeStatus = RobotStatus.OpModeStatus.STOPPED;
                        requestedOpModeStatus = RobotStatus.OpModeStatus.STOPPED;
                        opModeManager.stopActiveOpMode();
                        break;
                    }

                    case RETURN_VALUE: {
                        lastInputValue = message.getText();
                        break;
                    }

                    case SMARTDASHBOARD_GET: {
                        smartdashboardResponses.add(message.getText());
                        break;
                    }

                    case GAMEPAD_1_SET: {
                        try {
                            if (opModeManager == null || opModeManager.getActiveOpMode() == null)
                                return;
                            if (message.getText().equals("default")) {
                                if (opModeManager.getActiveOpMode().gamepad1 instanceof DashboardGamepad)
                                    opModeManager.getActiveOpMode().gamepad1 = ((OurEventLoop) eventLoop).getGamepads()[0];
                            } else if (opModeManager.getActiveOpMode().gamepad1 instanceof DashboardGamepad) {
                                ((DashboardGamepad) opModeManager.getActiveOpMode().gamepad1).update(message.getText());
                            } else {
                                opModeManager.getActiveOpMode().gamepad1 = new DashboardGamepad();
                                ((DashboardGamepad) opModeManager.getActiveOpMode().gamepad1).update(message.getText());
                            }
                        } catch (ClassCastException e) {
                        } catch (NullPointerException e) {
                        }

                        break;
                    }

                    case GAMEPAD_2_SET: {
                        try {
                            if (opModeManager == null || opModeManager.getActiveOpMode() == null)
                                return;
                            if (message.getText().equals("default")) {
                                if (opModeManager.getActiveOpMode().gamepad2 instanceof DashboardGamepad)
                                    opModeManager.getActiveOpMode().gamepad2 = ((OurEventLoop) eventLoop).getGamepads()[0];
                            } else if (opModeManager.getActiveOpMode().gamepad2 instanceof DashboardGamepad) {
                                ((DashboardGamepad) opModeManager.getActiveOpMode().gamepad2).update(message.getText());
                            } else {
                                opModeManager.getActiveOpMode().gamepad2 = new DashboardGamepad();
                                ((DashboardGamepad) opModeManager.getActiveOpMode().gamepad2).update(message.getText());
                            }
                        } catch (ClassCastException e) {
                        }

                        break;
                    }

                    default: {
                        break;
                    }
                }
            }

            ////////////////Keeping track of the state of the active opMode////////////////
            switch (activeOpModeStatus) {
                case INIT:
                    if (requestedOpModeStatus.equals(RobotStatus.OpModeStatus.RUNNING)) {
                        activeOpModeStatus = RobotStatus.OpModeStatus.RUNNING;
                        if (data != null && data.isConnected()) {
                            data.write(new Message(MessageType.ROBOT_STATUS, "RUNNING"));
                        }
                    }
                    if (requestedOpModeStatus.equals(RobotStatus.OpModeStatus.STOPPED)) {
                        activeOpModeStatus = RobotStatus.OpModeStatus.STOPPED;
                        if (data != null && data.isConnected()) {
                            data.write(new Message(MessageType.ROBOT_STATUS, "STOPPED"));
                        }
                    }
                    break;
                case RUNNING:
                    if (requestedOpModeStatus.equals(RobotStatus.OpModeStatus.STOPPED)) {
                        activeOpModeStatus = RobotStatus.OpModeStatus.STOPPED;
                        if (data != null && data.isConnected()) {
                            data.write(new Message(MessageType.ROBOT_STATUS, "STOPPED"));
                        }
                    }
                    break;
                case STOPPED:
                    if (requestedOpModeStatus.equals(RobotStatus.OpModeStatus.INIT)) {
                        if (data != null && data.isConnected()) {
                            activeOpModeStatus = RobotStatus.OpModeStatus.INIT;
                            data.write(new Message(MessageType.SELECT_OP_MODE, opModeManager.getActiveOpModeName()));
                            data.write(new Message(MessageType.ROBOT_STATUS, "INIT"));
                        }
                    }
                    break;
            }
        }
    }

    ////////////////Code to poll the robot battery voltage////////////////
    public String battery() {
        double sensors = 0;
        double voltage = 0;
        if (voltageSensors == null) {
            if (opModeManager == null) return "0";
            voltageSensors = opModeManager.getHardwareMap().getAll(VoltageSensor.class);
        }
        if (voltageSensors.size() < 1) return "0";
        for (VoltageSensor voltageSensor : voltageSensors) {
            try {
                if (voltageSensor.getVoltage() != 0) {
                    sensors++;
                    voltage += voltageSensor.getVoltage();
                }
            } catch (Exception e) {
                DashboardTelemtry.write("Robot Battery Voltage Read Error");
            }
        }
        return String.format("%.2f", voltage / sensors);
    }

    ////////////////Returns an instance of dashboardtelemetry to be used in the user code////////////////
    public dashboardtelemetry getTelemetry() {
        return DashboardTelemtry;
    }

    public smartdashboard getSmartDashboard() {
        return SmartDashboard;
    }

    public static String getCurrentOpMode() {
        return dashboard.internalGetCurrentOpMode();
    }

    public String internalGetCurrentOpMode() {
        return activeOpModeName;
    }

    ////////////////Called by the user code to request an input value from the dashboard, calls "internalGetInputValue()"////////////////
    public static double getInputValueDouble(String caption) {
        String value = dashboard.internalGetInputValue(caption);
        try {
            if (value.equals("")) return 0;
            return Double.valueOf(value);
        } catch (NumberFormatException e) {
            if (!caption.contains("Please enter a valid double for: "))
                caption = "Please enter a valid double for: " + caption;
            if (dashboard.requestedOpModeStatus == RobotStatus.OpModeStatus.STOPPED) return 0.0;
            return getInputValueDouble(caption);
        }
    }

    public static String getInputValueString(String caption) {
        return dashboard.internalGetInputValue(caption);
    }

    ////////////////The code to ask the dashboard to input a value and wait for a response////////////////
    ////////////////This can also be used to block the user opMode to do autonomous step by step////////////////
    private String internalGetInputValue(String caption) {
        if (data != null) data.write(new Message(MessageType.GET_VALUE, caption));
        lastInputValue = null;
        while (activeOpModeStatus == RobotStatus.OpModeStatus.STOPPED && isRunning && connected &&
                requestedOpModeStatus != RobotStatus.OpModeStatus.STOPPED && !Thread.currentThread().isInterrupted())
            ;
        while (data != null && lastInputValue == null && isRunning && connected &&
                activeOpModeStatus != RobotStatus.OpModeStatus.STOPPED && !Thread.currentThread().isInterrupted())
            ;
        try {
            if (lastInputValue != null) return lastInputValue;
        } catch (NumberFormatException e) {
        }
        return "";
    }

    public static void startOpMode(String name) {
        dashboard.internalStartOpMode(name);
    }

    private void internalStartOpMode(String name) {
        new Thread(() -> {
            while (!opModeManager.getActiveOpModeName().equals("$Stop$Robot$")) ;
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            opModeManager.initActiveOpMode(name);
            try {
                Thread.sleep(4000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            opModeManager.startActiveOpMode();
        }).start();
    }

    ////////////////Called by the "RobotRestartChecker" thread in "FtcRobotControllerActivity" to see if the////////////////
    ////////////////restart button on the dashboard has been pressed, calls "internalRobotRestartRequested()"////////////////
    public static boolean robotRestartRequested() {
        return dashboard.internalRobotRestartRequested();
    }

    ////////////////The actual code to check if the restart button has been pressed////////////////
    private boolean internalRobotRestartRequested() {
        if (restartRequested) {
            restartRequested = false;
            return true;
        }
        return false;
    }

    ////////////////Called by "FtcRobotControllerActivity" when a restart has completed, calls "internalRestartComplete()"////////////////
    public static void restartComplete() {
        dashboard.internalRestartComplete();
    }

    ////////////////Code to refresh the dashboard when a restart is completed////////////////
    private void internalRestartComplete() {
        if (data != null) {
            data.write(new Message(MessageType.RESTART_ROBOT, "HI"));
            voltageSensors = null;
        }
    }

    ////////////////Called by "opModeManagerImpl"////////////////
    @Override
    public void onOpModePreInit(OpMode opMode) {
        if (!dashboard.opModeManager.getActiveOpModeName().equals("$Stop$Robot$")) {
            activeOpModeName = dashboard.opModeManager.getActiveOpModeName();
            requestedOpModeStatus = RobotStatus.OpModeStatus.INIT;
            opModeInitTime = System.currentTimeMillis();
        }
    }

    ////////////////Called by "opModeManagerImpl"////////////////
    @Override
    public void onOpModePreStart(OpMode opMode) {
        if (!dashboard.opModeManager.getActiveOpModeName().equals("$Stop$Robot$")) {
            requestedOpModeStatus = RobotStatus.OpModeStatus.RUNNING;
            opModeStartTime = System.currentTimeMillis();
        }
    }

    ////////////////Called by "AbstractOpMode"////////////////
    public static void onOpModePreStop() {
        dashboard.internalOnOpModePreStop();
    }

    ////////////////Called by "onOpModePreStop"////////////////
    public void internalOnOpModePreStop() {
    }

    ////////////////Called by "opModeManagerImpl"////////////////
    @Override
    public void onOpModePostStop(OpMode opMode) {
        if (!dashboard.opModeManager.getActiveOpModeName().equals("$Stop$Robot$"))
            requestedOpModeStatus = RobotStatus.OpModeStatus.STOPPED;
    }

    public int internalGetTimeSinceInit() {
        return (int)(System.currentTimeMillis() - opModeInitTime);
    }

    public static int getTimeSinceInit() {
        return dashboard.internalGetTimeSinceInit();
    }

    public int internalGetTimeSinceStart() {
        return (int)(System.currentTimeMillis() - opModeStartTime);
    }

    public static int getTimeSinceStart() {
        return dashboard.internalGetTimeSinceStart();
    }

    public String getLogPreMessage() {
        return dashboard.internalGetLogPreMessage();
    }

    private String internalGetLogPreMessage() {
        String packet = "";

        if (requestedOpModeStatus.equals(RobotStatus.OpModeStatus.RUNNING) ||
                activeOpModeStatus.equals(RobotStatus.OpModeStatus.RUNNING)) {
            double currentTime = internalGetTimeSinceStart() / 1000.0;
            packet = date.getDateTime().replace(": ", "") + "%&&%&&%Time since start: " + currentTime + "%&&%&&%";
        } else if (requestedOpModeStatus.equals(RobotStatus.OpModeStatus.INIT)) {
            double currentTime = internalGetTimeSinceInit() / 1000.0;
            packet = date.getDateTime().replace(": ", "") + "%&&%&&%Time since init: " + currentTime + "%&&%&&%";
        } else packet = date.getDateTime().replace(": ", "") + "%&&%&&%Stopped%&&%&&%";

        return packet;
    }

    private String internalGetTelemetryPreMessage() {
        String packet = "";

        if (requestedOpModeStatus.equals(RobotStatus.OpModeStatus.RUNNING) ||
                activeOpModeStatus.equals(RobotStatus.OpModeStatus.RUNNING)) {
            double currentTime = getTimeSinceStart() / 1000.0;
            packet = date.getDateTime() + "`Time since start: " + currentTime + "`";
        } else if (requestedOpModeStatus.equals(RobotStatus.OpModeStatus.INIT)) {
            double currentTime = getTimeSinceInit() / 1000.0;
            packet = date.getDateTime() + "`Time since init: " + currentTime + "`";
        } else packet = date.getDateTime() + "`Stopped`";

        return packet;
    }

    ////////////////Thread to start up dashboardtelemetry and then call "receiveMessage()" in a loop////////////////
    private class DashboardHandler implements Runnable {

        public DashboardHandler() {
        }

        @Override
        public void run() {
            DashboardTelemtry = new dashboardtelemetry();
            SmartDashboard = new smartdashboard();
            while (isRunning) {
                receiveMessage();
            }
        }
    }

    ////////////////Starts a connection and continuously monitor it////////////////
    private class DataHandler implements Runnable {

        public DataHandler() {
        }

        int loop = 0;

        @Override
        public void run() {
            while (data == null) {
                data = new Data();
            }
            while (isRunning) {
                if (data.isConnected()) {
                    if (data != null && loop == 0 && !restartRequested)
                        data.write(new Message(MessageType.BATTERY, battery()));
                    if (batteryChecker != null && loop % 2 == 0)
                        batteryChecker.pollBatteryLevel(batteryWatcher);
                    if (!connected) {
                        connected = true;
                    }
                } else {
                    if (connected) {
                        connected = false;
                        data = new Data();
                    }
                }
                loop++;
                if (loop > 40) loop = 0;
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    ////////////////Used by user OpMode to display telemetry on the dashboard////////////////
    public class dashboardtelemetry {

        public dashboardtelemetry() {

        }

        public void write(String text) {
            String packet = internalGetTelemetryPreMessage() + text;

            if (data != null && connected) data.write(new Message(MessageType.TELEMETRY, packet));
            else oldTelemetry += "&#%#&" + packet;
        }

        public void updateInfo() {
            if (data != null && connected) {
                data.write(new Message(MessageType.INFO, infoText));
            }
            infoText = "";
        }

        public void info(String text) {
            infoText = infoText + text + "!#@$";
        }

        public void info(int text) {
            info(String.valueOf(text));
        }

        public void info(double text) {
            info(String.valueOf(text));
        }

        public void putString(String key, String value) {
            sharedPreferences.edit().putString(key, value).apply();
        }

        public void putInt(String key, int value) {
            sharedPreferences.edit().putInt(key, value).apply();
        }

        public void putFloat(String key, float value) {
            sharedPreferences.edit().putFloat(key, value).apply();
        }

        public void putBoolean(String key, boolean value) {
            sharedPreferences.edit().putBoolean(key, value).apply();
        }

        public String getString(String key, String defaultValue) {
            return sharedPreferences.getString(key, defaultValue);
        }

        public int getInt(String key, int defaultValue) {
            return sharedPreferences.getInt(key, defaultValue);
        }

        public float getFloat(String key, float defaultValue) {
            return sharedPreferences.getFloat(key, defaultValue);
        }

        public boolean getBoolean(String key, boolean defaultValue) {
            return sharedPreferences.getBoolean(key, defaultValue);
        }
    }

    public class smartdashboard {

        public smartdashboard() {

        }

        private void put(String text) {
            if (data != null && connected)
                data.write(new Message(MessageType.SMARTDASHBOARD_PUT, text));
            else oldSmartdashboard += text + "#&%&#";
        }

        public void putValue(Object key, Object value) {
            put("VALUE<&#%#&>" + String.valueOf(key) + "<&#%#&>" + String.valueOf(value));
        }

        public void putBoolean(Object key, boolean value) {
            put("BOOLEAN<&#%#&>" + String.valueOf(key) + "<&#%#&>" + String.valueOf(value));
        }

        public void putButton(Object key) {
            put("BUTTON<&#%#&>" + String.valueOf(key));
        }

        public void putInput(Object key) {
            put("INPUT<&#%#&>" + String.valueOf(key));
        }

        public void putSlider(Object key, int low, int high) {
            put("SLIDER<&#%#&>" + String.valueOf(key) + "<&#%#&>" + low + "<&#%#&>" + high);
        }

        public void putGraph(Object key, String set, double x, double y) {
            put("GRAPH<&#%#&>" + String.valueOf(key) + "<&#%#&>" + set + "<&#%#&>" + x + "<&#%#&>" + y);
        }

        public void putGraphPoint(Object key, String set, double x, double y) {
            put("GRAPH_POINT<&#%#&>" + String.valueOf(key) + "<&#%#&>" + set + "<&#%#&>" + x + "<&#%#&>" + y);
        }

        public void putGraphCircle(Object key, String set, double r, double x, double y) {
            put("GRAPH_CIRCLE<&#%#&>" + String.valueOf(key) + "<&#%#&>" + set + "<&#%#&>" + r + "<&#%#&>" + x + "<&#%#&>" + y);
        }

        public void clearGraph(Object key, String set) {
            put("GRAPH_CLEAR<&#%#&>" + String.valueOf(key) + "<&#%#&>" + set);
        }

        public void get(String text) {
            if (data != null && connected)
                data.write(new Message(MessageType.SMARTDASHBOARD_GET, text));
        }

        public String getValue(Object key) {

            int requestID = -1;

            synchronized (smartdashboardRequestIDLock) {
                get(smartdashboardRequestID + "<&#%#&>VALUE<&#%#&>" + String.valueOf(key));
                requestID = smartdashboardRequestID;
                smartdashboardRequestID++;
            }

            String response = null;

            loop:
            while (connected) {
                synchronized (smartdashboardResponses) {
                    for (String message : smartdashboardResponses) {
                        String[] parts = message.split("<&#%#&>");
                        if (Integer.valueOf(parts[0]) == requestID) {
                            response = message;
                            break loop;
                        }
                    }
                }
            }

            smartdashboardResponses.remove(response);

            response = response.split("<&#%#&>")[1];

            return response;
        }

        public Boolean getBoolean(Object key) {

            int requestID = -1;

            synchronized (smartdashboardRequestIDLock) {
                get(smartdashboardRequestID + "<&#%#&>BOOLEAN<&#%#&>" + String.valueOf(key));
                requestID = smartdashboardRequestID;
                smartdashboardRequestID++;
            }

            String response = null;

            loop:
            while (connected) {
                synchronized (smartdashboardResponses) {
                    for (String message : smartdashboardResponses) {
                        String[] parts = message.split("<&#%#&>");
                        if (Integer.valueOf(parts[0]) == requestID) {
                            response = message;
                            break loop;
                        }
                    }
                }
            }

            smartdashboardResponses.remove(response);

            response = response.split("<&#%#&>")[1];

            return Boolean.valueOf(response);
        }

        public Boolean getButton(Object key) {

            int requestID = -1;

            synchronized (smartdashboardRequestIDLock) {
                get(smartdashboardRequestID + "<&#%#&>BUTTON<&#%#&>" + String.valueOf(key));
                requestID = smartdashboardRequestID;
                smartdashboardRequestID++;
            }

            String response = null;

            loop:
            while (connected) {
                synchronized (smartdashboardResponses) {
                    for (String message : smartdashboardResponses) {
                        String[] parts = message.split("<&#%#&>");
                        if (Integer.valueOf(parts[0]) == requestID) {
                            response = message;
                            break loop;
                        }
                    }
                }
            }

            smartdashboardResponses.remove(response);

            response = response.split("<&#%#&>")[1];

            return Boolean.valueOf(response);
        }

        public String getInput(Object key) {

            int requestID = -1;

            synchronized (smartdashboardRequestIDLock) {
                get(smartdashboardRequestID + "<&#%#&>INPUT<&#%#&>" + String.valueOf(key));
                requestID = smartdashboardRequestID;
                smartdashboardRequestID++;
            }

            String response = null;

            loop:
            while (connected) {
                synchronized (smartdashboardResponses) {
                    for (String message : smartdashboardResponses) {
                        String[] parts = message.split("<&#%#&>");
                        if (Integer.valueOf(parts[0]) == requestID) {
                            response = message;
                            break loop;
                        }
                    }
                }
            }

            smartdashboardResponses.remove(response);

            response = response.split("<&#%#&>")[1];

            return response;
        }

        public int getSlider(Object key) {

            int requestID = -1;

            synchronized (smartdashboardRequestIDLock) {
                get(smartdashboardRequestID + "<&#%#&>SLIDER<&#%#&>" + String.valueOf(key));
                requestID = smartdashboardRequestID;
                smartdashboardRequestID++;
            }

            String response = null;

            loop:
            while (connected) {
                synchronized (smartdashboardResponses) {
                    for (String message : smartdashboardResponses) {
                        String[] parts = message.split("<&#%#&>");
                        if (Integer.valueOf(parts[0]) == requestID) {
                            response = message;
                            break loop;
                        }
                    }
                }
            }

            smartdashboardResponses.remove(response);

            response = response.split("<&#%#&>")[1];

            return Integer.valueOf(response);
        }
    }

    ////////////////Updates the phone battery level indicator on the dashboard////////////////
    ////////////////Called automatically by the driver station battery level code////////////////
    @Override
    public void updateBatteryStatus(BatteryChecker.BatteryStatus status) {
        if (data != null && status != null) {
            data.write(new Message(MessageType.PHONE_BATTERY, String.valueOf(status.percent)));
        }
    }
}