package org.firstinspires.ftc.teamcode.util.hardware;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * describes everything about robot config (FTC SDK 12.0) and what you'd see on the ds:
 * includes:
 * <ul>
 *   <li>all hubs over RS485</li>
 *   <li>Expansion Hubs connected over USB ({@link #usbPortal})</li>
 *   <li>motor, servo, analog, PWM, digital and I2C ports on each hub</li>
 *   <li>webcams and ethernet devices such as a Limelight</li>
 * </ul>
 *
 * mistakes r collected so no crash
 */
public final class HardwareConfig {

    /** The kind of port a device plugs into. */
    public enum Kind {
        MOTOR("motor"), SERVO("servo"), ANALOG("analog"), PWM("pwm"), DIGITAL("digital"), I2C("i2c");

        final String label;

        Kind(String label) {
            this.label = label;
        }
    }

    /**
     * grouped xml tags
     */
    public static final class Type {
        // motor ports
        /** goBILDA 5202-4 series. */
        public static final String GOBILDA_5202 = "goBILDA5202SeriesMotor";
        public static final String GOBILDA_5201 = "goBILDA5201SeriesMotor";
        public static final String REV_CORE_HEX = "RevRoboticsCoreHexMotor";
        public static final String REV_HD_HEX_20 = "RevRobotics20HDHexMotor";
        public static final String REV_HD_HEX_40 = "RevRobotics40HDHexMotor";
        public static final String REV_ULTRAPLANETARY = "RevRoboticsUltraplanetaryHDHexMotor";
        public static final String NEVEREST_3_7 = "NeveRest3.7v1Gearmotor";
        public static final String NEVEREST_20 = "NeveRest20Gearmotor";
        public static final String NEVEREST_40 = "NeveRest40Gearmotor";
        public static final String NEVEREST_60 = "NeveRest60Gearmotor";
        public static final String STUDICA_MAVERICK = "StudicaMaverick";
        public static final String TETRIX_MOTOR = "TetrixMotor";
        public static final String MATRIX_12V = "Matrix12vMotor";
        public static final String UNSPECIFIED_MOTOR = "Motor";

        // servo ports
        public static final String SERVO = "Servo";
        public static final String CR_SERVO = "ContinuousRotationServo";
        public static final String REV_SPARK_MINI = "RevSPARKMini";
        public static final String REV_BLINKIN = "RevBlinkinLedDriver";

        // analog ports
        public static final String ANALOG_INPUT = "AnalogInput";
        public static final String MR_ANALOG_TOUCH = "ModernRoboticsAnalogTouchSensor";
        public static final String MR_OPTICAL_DISTANCE = "OpticalDistanceSensor";

        // pwm ports
        public static final String PWM_OUTPUT = "PulseWidthDevice";

        // digital ports
        public static final String DIGITAL_DEVICE = "DigitalDevice";
        public static final String LED = "Led";
        public static final String REV_TOUCH = "RevTouchSensor";

        // i2c buses
        /** imu built into newer Control Hubs (BHI260AP) is on bus 0 */
        public static final String CONTROL_HUB_IMU_BHI260 = "ControlHubImuBHI260AP";
        /** same for older Control/Expansion Hubs (BNO055) */
        public static final String REV_IMU_BNO055 = "LynxEmbeddedIMU";
        /** REV 9-axis IMU (external) */
        public static final String REV_EXTERNAL_IMU = "RevExternalImu";
        public static final String ADAFRUIT_BNO055_IMU = "AdafruitBNO055IMU";
        public static final String GOBILDA_PINPOINT = "goBILDAPinpoint";
        public static final String SPARKFUN_OTOS = "SparkFunOTOS";
        public static final String OCTOQUAD = "OctoQuadFTC";
        public static final String REV_COLOR_V3 = "RevColorSensorV3";
        /** REV Color Sensor V2 */
        public static final String REV_COLOR_V2 = "LynxColorSensor";
        public static final String REV_2M_DISTANCE = "REV_VL53L0X_RANGE_SENSOR";
        public static final String HUSKYLENS = "HuskyLens";
        public static final String NAVX_MICRO = "KauaiLabsNavxMicro";
        public static final String MAXSONAR_I2CXL = "MaxSonarI2CXL";
        public static final String SPARKFUN_LED_STICK = "QWIIC_LED_STICK";
        public static final String ANDYMARK_COLOR = "AndyMarkColor";
        public static final String ANDYMARK_IMU = "AndyMarkIMU";
        public static final String ANDYMARK_TOF = "AndyMarkTOF";
        public static final String ADAFRUIT_COLOR = "AdafruitColorSensor";
        public static final String MR_COLOR = "ColorSensor";
        public static final String MR_GYRO = "Gyro";
        public static final String MR_IR_SEEKER_V3 = "IrSeekerV3";
        public static final String MR_COMPASS = "ModernRoboticsI2cCompassSensor";
        public static final String MR_RANGE = "ModernRoboticsI2cRangeSensor";

        private Type() {}
    }

    private static final Map<String, Kind> SDK_TYPES = new LinkedHashMap<>();

    static {
        sdk(Kind.MOTOR, Type.GOBILDA_5202, Type.GOBILDA_5201, Type.REV_CORE_HEX, Type.REV_HD_HEX_20,
                Type.REV_HD_HEX_40, Type.REV_ULTRAPLANETARY, Type.NEVEREST_3_7, Type.NEVEREST_20,
                Type.NEVEREST_40, Type.NEVEREST_60, Type.STUDICA_MAVERICK, Type.TETRIX_MOTOR,
                Type.MATRIX_12V, Type.UNSPECIFIED_MOTOR);
        sdk(Kind.SERVO, Type.SERVO, Type.CR_SERVO, Type.REV_SPARK_MINI, Type.REV_BLINKIN);
        sdk(Kind.ANALOG, Type.ANALOG_INPUT, Type.MR_ANALOG_TOUCH, Type.MR_OPTICAL_DISTANCE);
        sdk(Kind.PWM, Type.PWM_OUTPUT);
        sdk(Kind.DIGITAL, Type.DIGITAL_DEVICE, Type.LED, Type.REV_TOUCH);
        sdk(Kind.I2C, Type.CONTROL_HUB_IMU_BHI260, Type.REV_IMU_BNO055, Type.REV_EXTERNAL_IMU,
                Type.ADAFRUIT_BNO055_IMU, Type.GOBILDA_PINPOINT, Type.SPARKFUN_OTOS, Type.OCTOQUAD,
                Type.REV_COLOR_V3, Type.REV_COLOR_V2, Type.REV_2M_DISTANCE, Type.HUSKYLENS,
                Type.NAVX_MICRO, Type.MAXSONAR_I2CXL, Type.SPARKFUN_LED_STICK, Type.ANDYMARK_COLOR,
                Type.ANDYMARK_IMU, Type.ANDYMARK_TOF, Type.ADAFRUIT_COLOR, Type.MR_COLOR, Type.MR_GYRO,
                Type.MR_IR_SEEKER_V3, Type.MR_COMPASS, Type.MR_RANGE);
    }

    private static void sdk(Kind kind, String... tags) {
        for (String tag : tags) SDK_TYPES.put(tag, kind);
    }

    /** type of port or null */
    public static Kind sdkKind(String tag) {
        return SDK_TYPES.get(tag);
    }

    // hardware limits from  SDK LynxConstants
    public static final int CONTROL_HUB_ADDRESS = 173;
    public static final int MAX_MODULE_ADDRESS = 10;
    public static final int MOTOR_PORTS = 4;
    public static final int SERVO_PORTS = 6;
    public static final int ANALOG_PORTS = 4;
    public static final int PWM_PORTS = 4;
    public static final int DIGITAL_PORTS = 8;
    public static final int I2C_BUSES = 4;

    static final String EMBEDDED_SERIAL = "(embedded)";
    static final String CONTROL_HUB_PORTAL = "Control Hub Portal";
    static final String CONTROL_HUB = "Control Hub";

    public interface HubSetup {
        void apply(Hub hub);
    }

    public interface ServoHubSetup {
        void apply(ServoHub hub);
    }

    public interface PortalSetup {
        void apply(Portal portal);
    }

    private final String name;
    private final List<Object> topLevel = new ArrayList<>(); // Portal, Webcam, EthernetDevice
    private Portal embedded;
    private int usbPortals;

    private final List<String> errors = new ArrayList<>();
    private final Map<String, String> names = new HashMap<>(); // device name -> usage
    private final List<Device> devices = new ArrayList<>();

    public HardwareConfig(String name) {
        this.name = name;
        if (name == null || name.trim().isEmpty()) {
            error("config is missing a name");
        } else if (name.matches(".*[\\\\/:*?\"<>|].*") || name.endsWith(".xml") || !name.equals(name.trim())) {
            error("\"" + name + "\" isn't a usable config name (needs to be file-valid)");
        }
    }

    public String name() {
        return name;
    }

    // ---------------------------------------------------------------------------------------------
    // building

    public HardwareConfig controlHub(HubSetup setup) {
        embedded().hub(CONTROL_HUB, CONTROL_HUB_ADDRESS, setup);
        return this;
    }

    /** exp hub connected to the Control Hub over RS485 -> usually 2 */
    public HardwareConfig expansionHub(int address, HubSetup setup) {
        embedded().expansionHub(address, setup);
        return this;
    }

    /** REV Servo Hub connected to the Control Hub over RS485 */
    public HardwareConfig servoHub(int address, ServoHubSetup setup) {
        embedded().servoHub(address, setup);
        return this;
    }

    /** RS485 hub on the Control Hub with a custom name */
    public HardwareConfig hub(String hubName, int address, HubSetup setup) {
        embedded().hub(hubName, address, setup);
        return this;
    }

    /** RS485 servo hub on the Control Hub with a custom name */
    public HardwareConfig servoHub(String hubName, int address, ServoHubSetup setup) {
        embedded().servoHub(hubName, address, setup);
        return this;
    }

    /**
     * expansion hub over usb + any chained
     */
    public HardwareConfig usbPortal(String serialNumber, int parentAddress, PortalSetup setup) {
        return usbPortal("Expansion Hub Portal " + (usbPortals + 1), serialNumber, parentAddress, setup);
    }

    public HardwareConfig usbPortal(String portalName, String serialNumber, int parentAddress, PortalSetup setup) {
        usbPortals++;
        if (serialNumber == null || serialNumber.trim().isEmpty() || serialNumber.equals(EMBEDDED_SERIAL)) {
            error(portalName + ": needs the hub's USB serial number (shown in the RC's configure menu)");
        }
        Portal portal = new Portal(this, portalName, serialNumber, parentAddress, false);
        claimName(portalName, "USB portal");
        topLevel.add(portal);
        setup.apply(portal);
        return this;
    }

    /** USB webcam */
    public HardwareConfig webcam(String deviceName, String serialNumber) {
        claimName(deviceName, "webcam");
        if (serialNumber == null || serialNumber.trim().isEmpty()) error("webcam \"" + deviceName + "\" needs a serial number");
        topLevel.add(new Webcam(deviceName, serialNumber));
        return this;
    }

    /**
     * ethernet-over-USB device such as Limelight; {@code serialNumber} looks like
     * {@code EthernetOverUsb:eth0:172.29.0.29}; easiest is to configure it once on the RC and pull
     */
    public HardwareConfig ethernetDevice(String deviceName, String serialNumber, String ipAddress) {
        claimName(deviceName, "ethernet device");
        if (serialNumber == null || !serialNumber.matches("EthernetOverUsb:[^:]+:" + IPV4)) {
            error("ethernet device \"" + deviceName + "\": serial number should look like EthernetOverUsb:eth0:172.29.0.29, got \"" + serialNumber + "\"");
        }
        if (ipAddress == null || !ipAddress.matches(IPV4)) {
            error("ethernet device \"" + deviceName + "\": \"" + ipAddress + "\" isn't an IPv4 address");
        }
        topLevel.add(new EthernetDevice(deviceName, serialNumber, ipAddress));
        return this;
    }

    private static final String IPV4 = "((25[0-5]|2[0-4]\\d|1?\\d?\\d)\\.){3}(25[0-5]|2[0-4]\\d|1?\\d?\\d)";

    private Portal embedded() {
        if (embedded == null) {
            embedded = new Portal(this, CONTROL_HUB_PORTAL, EMBEDDED_SERIAL, CONTROL_HUB_ADDRESS, true);
            claimName(CONTROL_HUB_PORTAL, "Control Hub portal");
            topLevel.add(embedded);
        }
        return embedded;
    }

    void error(String message) {
        errors.add(message);
    }

    void claimName(String deviceName, String where) {
        if (deviceName == null || deviceName.trim().isEmpty()) {
            error(where + " has no name");
            return;
        }
        if (!deviceName.equals(deviceName.trim())) {
            error(where + ": name \"" + deviceName + "\" has leading/trailing spaces (hardwareMap.get would need them too)");
        }
        String previous = names.get(deviceName);
        if (previous != null) {
            error("name \"" + deviceName + "\" is used twice: " + previous + " and " + where);
        } else {
            names.put(deviceName, where);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // model

    static final class Device {
        final Kind kind;     // null for device(...)
        final int port;
        final int bus;       // i2c only
        final String name;
        final String type;
        final String where;

        Device(Kind kind, int port, int bus, String name, String type, String where) {
            this.kind = kind;
            this.port = port;
            this.bus = bus;
            this.name = name;
            this.type = type;
            this.where = where;
        }
    }

    /** anything on rs485. */
    public abstract static class Module<SELF extends Module<SELF>> {
        final HardwareConfig root;
        final String name;
        final int address;
        final Device[] servos = new Device[SERVO_PORTS];

        Module(HardwareConfig root, String name, int address) {
            this.root = root;
            this.name = name;
            this.address = address;
        }

        abstract SELF self();

        abstract String xmlTag();

        public SELF servo(int port, String deviceName) {
            return servo(port, deviceName, Type.SERVO);
        }

        public SELF crServo(int port, String deviceName) {
            return servo(port, deviceName, Type.CR_SERVO);
        }

        /** on servo port: {@link Type#SERVO}, {@link Type#REV_SPARK_MINI}, {@link Type#REV_BLINKIN}... */
        public SELF servo(int port, String deviceName, String type) {
            put(servos, Kind.SERVO, port, deviceName, type);
            return self();
        }

        void put(Device[] slots, Kind kind, int port, String deviceName, String type) {
            String where = name + " " + kind.label + " port " + port;
            if (port < 0 || port >= slots.length) {
                root.error(where + " (\"" + deviceName + "\"): " + kind.label + " ports are 0-" + (slots.length - 1));
                return;
            }
            if (slots[port] != null) {
                root.error(where + " has two devices: \"" + slots[port].name + "\" and \"" + deviceName + "\"");
                return;
            }
            slots[port] = root.newDevice(kind, port, -1, deviceName, type, where);
        }

        boolean isEmpty() {
            for (Device d : servos) if (d != null) return false;
            return true;
        }

        List<Device> all() {
            List<Device> out = new ArrayList<>();
            for (Device d : servos) if (d != null) out.add(d);
            return out;
        }
    }

    /** cHub or eHub */
    public static final class Hub extends Module<Hub> {
        final Device[] motors = new Device[MOTOR_PORTS];
        final Device[] analog = new Device[ANALOG_PORTS];
        final Device[] pwm = new Device[PWM_PORTS];
        final Device[] digital = new Device[DIGITAL_PORTS];
        final List<List<Device>> i2c = new ArrayList<>();
        final List<Device> other = new ArrayList<>();

        Hub(HardwareConfig root, String name, int address) {
            super(root, name, address);
            for (int i = 0; i < I2C_BUSES; i++) i2c.add(new ArrayList<Device>());
        }

        @Override Hub self() {
            return this;
        }

        @Override String xmlTag() {
            return "LynxModule";
        }

        public Hub motor(int port, String deviceName, String type) {
            put(motors, Kind.MOTOR, port, deviceName, type);
            return this;
        }

        public Hub analog(int port, String deviceName) {
            return analog(port, deviceName, Type.ANALOG_INPUT);
        }

        public Hub analog(int port, String deviceName, String type) {
            put(analog, Kind.ANALOG, port, deviceName, type);
            return this;
        }

        public Hub pwm(int port, String deviceName) {
            return pwm(port, deviceName, Type.PWM_OUTPUT);
        }

        public Hub pwm(int port, String deviceName, String type) {
            put(pwm, Kind.PWM, port, deviceName, type);
            return this;
        }

        public Hub digital(int port, String deviceName) {
            return digital(port, deviceName, Type.DIGITAL_DEVICE);
        }

        public Hub digital(int port, String deviceName, String type) {
            put(digital, Kind.DIGITAL, port, deviceName, type);
            return this;
        }

        /** I2C device on {@code bus} (0-3) */
        public Hub i2c(int bus, String deviceName, String type) {
            String where = name + " i2c bus " + bus;
            if (bus < 0 || bus >= I2C_BUSES) {
                root.error(where + " (\"" + deviceName + "\"): i2c buses are 0-" + (I2C_BUSES - 1));
                return this;
            }
            List<Device> list = i2c.get(bus);
            // port = device position on its bus
            list.add(root.newDevice(Kind.I2C, list.size(), bus, deviceName, type, where));
            return this;
        }

        /**
         * writes {@code <type name=".." port=".." />} as-is, with no checks on the
         * type or port. prefer the methods above (last resort)
         */
        public Hub device(int port, String deviceName, String type) {
            other.add(root.newDevice(null, port, -1, deviceName, type, name + " port " + port));
            return this;
        }

        @Override boolean isEmpty() {
            return all().isEmpty();
        }

        @Override List<Device> all() {
            List<Device> out = new ArrayList<>();
            for (Device[] slots : new Device[][] {motors, servos, analog, pwm, digital}) {
                for (Device d : slots) if (d != null) out.add(d);
            }
            for (List<Device> bus : i2c) out.addAll(bus);
            out.addAll(other);
            return out;
        }
    }

    /** REV Servo Hub (6 servo ports) */
    public static final class ServoHub extends Module<ServoHub> {
        ServoHub(HardwareConfig root, String name, int address) {
            super(root, name, address);
        }

        @Override ServoHub self() {
            return this;
        }

        @Override String xmlTag() {
            return "ServoHub";
        }
    }

    /** USB connection to a chain of RS485 (the cHub's built-in one or USB eHub). */
    public static final class Portal {
        final HardwareConfig root;
        final String name;
        final String serialNumber;
        final int parentAddress;
        final boolean embedded;
        final List<Module<?>> modules = new ArrayList<>();

        Portal(HardwareConfig root, String name, String serialNumber, int parentAddress, boolean embedded) {
            this.root = root;
            this.name = name;
            this.serialNumber = serialNumber;
            this.parentAddress = parentAddress;
            this.embedded = embedded;
        }

        public Portal expansionHub(int address, HubSetup setup) {
            return hub("Expansion Hub " + address, address, setup);
        }

        public Portal hub(String hubName, int address, HubSetup setup) {
            Hub hub = new Hub(root, hubName, address);
            add(hub);
            setup.apply(hub);
            return this;
        }

        public Portal servoHub(int address, ServoHubSetup setup) {
            return servoHub("Servo Hub " + address, address, setup);
        }

        public Portal servoHub(String hubName, int address, ServoHubSetup setup) {
            ServoHub hub = new ServoHub(root, hubName, address);
            add(hub);
            setup.apply(hub);
            return this;
        }

        private void add(Module<?> module) {
            root.claimName(module.name, "hub at address " + module.address);
            boolean controlHub = embedded && module.address == CONTROL_HUB_ADDRESS && module instanceof Hub;
            if (!controlHub && (module.address < 1 || module.address > MAX_MODULE_ADDRESS)) {
                root.error(module.name + ": address " + module.address + " is invalid, hub addresses are 1-"
                        + MAX_MODULE_ADDRESS + (embedded ? " (" + CONTROL_HUB_ADDRESS + " is the Control Hub itself)" : ""));
            }
            for (Module<?> m : modules) {
                if (m.address == module.address) {
                    root.error(name + ": \"" + m.name + "\" and \"" + module.name + "\" both use address " + module.address);
                }
            }
            modules.add(module);
        }
    }

    static final class Webcam {
        final String name;
        final String serialNumber;

        Webcam(String name, String serialNumber) {
            this.name = name;
            this.serialNumber = serialNumber;
        }
    }

    static final class EthernetDevice {
        final String name;
        final String serialNumber;
        final String ipAddress;

        EthernetDevice(String name, String serialNumber, String ipAddress) {
            this.name = name;
            this.serialNumber = serialNumber;
            this.ipAddress = ipAddress;
        }
    }

    Device newDevice(Kind kind, int port, int bus, String deviceName, String type, String where) {
        claimName(deviceName, where);
        if (type == null || type.trim().isEmpty()) {
            error(where + " (\"" + deviceName + "\") has no device type");
        } else if (kind != null) {
            Kind sdk = SDK_TYPES.get(type);
            if (sdk != null && sdk != kind) {
                error(where + " (\"" + deviceName + "\"): " + type + " goes on a " + sdk.label + " port, not a " + kind.label + " port");
            }
        }
        Device d = new Device(kind, port, bus, deviceName, type, where);
        devices.add(d);
        return d;
    }

    // ---------------------------------------------------------------------------------------------
    // validation

    /** thrown by {@link #toXml} when config has mistakes */
    public static final class ConfigException extends RuntimeException {
        public final List<String> errors;

        ConfigException(List<String> errors) {
            super(join(errors, "\n  "));
            this.errors = errors;
        }
    }

    /**
     * check evert and return warnings + errors
     */
    public List<String> validate(Map<String, Kind> customTypes, List<String> errorsOut) {
        List<String> warnings = new ArrayList<>();
        errorsOut.addAll(errors);

        for (Device d : devices) {
            if (d.kind == null || d.type == null || SDK_TYPES.containsKey(d.type)) continue;
            Kind custom = customTypes.get(d.type);
            if (custom == null) {
                errorsOut.add(d.where + " (\"" + d.name + "\"): unknown device type \"" + d.type + "\"" + suggest(d.type, customTypes));
            } else if (custom != d.kind) {
                errorsOut.add(d.where + " (\"" + d.name + "\"): " + d.type + " goes on a " + custom.label + " port, not a " + d.kind.label + " port");
            }
        }
        for (Device d : devices) {
            if (d.kind == null) warnings.add(d.where + " (\"" + d.name + "\"): " + d.type + " was added with device(), so it isn't checked");
        }

        for (Object top : topLevel) {
            if (!(top instanceof Portal)) continue;
            Portal portal = (Portal) top;
            if (!portal.embedded && !portal.modules.isEmpty()) {
                boolean found = false;
                for (Module<?> m : portal.modules) found |= m.address == portal.parentAddress;
                if (!found) {
                    errorsOut.add(portal.name + ": no hub at address " + portal.parentAddress + ", the hub plugged in over USB");
                }
            }
            for (Module<?> m : portal.modules) {
                if (!(m instanceof Hub)) continue;
                List<List<Device>> buses = ((Hub) m).i2c;
                for (int bus = 0; bus < buses.size(); bus++) {
                    Map<String, Integer> count = new HashMap<>();
                    for (Device d : buses.get(bus)) {
                        Integer n = count.get(d.type);
                        count.put(d.type, n == null ? 1 : n + 1);
                        boolean imu = Type.CONTROL_HUB_IMU_BHI260.equals(d.type) || Type.REV_IMU_BNO055.equals(d.type);
                        if (imu && bus != 0) warnings.add(d.where + " (\"" + d.name + "\"): the built-in IMU is on i2c bus 0");
                    }
                    for (Map.Entry<String, Integer> e : count.entrySet()) {
                        if (e.getValue() > 1) {
                            warnings.add(m.name + " i2c bus " + bus + " has " + e.getValue() + " " + e.getKey()
                                    + " devices; identical sensors usually share an I2C address and can't be on the same bus");
                        }
                    }
                }
            }
        }
        return warnings;
    }

    private static String suggest(String type, Map<String, Kind> customTypes) {
        List<String> known = new ArrayList<>(SDK_TYPES.keySet());
        known.addAll(customTypes.keySet());
        String best = null;
        int bestDistance = Integer.MAX_VALUE;
        for (String k : known) {
            if (k.equalsIgnoreCase(type)) return ", did you mean \"" + k + "\"? (tags are case sensitive)";
            int dist = distance(k.toLowerCase(), type.toLowerCase());
            if (dist < bestDistance) {
                bestDistance = dist;
                best = k;
            }
        }
        if (best != null && bestDistance <= Math.max(2, type.length() / 4)) return ", did you mean \"" + best + "\"?";
        return " (custom device types need an @I2cDeviceType/@MotorType/... class in TeamCode)";
    }

    private static int distance(String a, String b) {
        int[] prev = new int[b.length() + 1];
        int[] cur = new int[b.length() + 1];
        for (int j = 0; j <= b.length(); j++) prev[j] = j;
        for (int i = 1; i <= a.length(); i++) {
            cur[0] = i;
            for (int j = 1; j <= b.length(); j++) {
                int sub = prev[j - 1] + (a.charAt(i - 1) == b.charAt(j - 1) ? 0 : 1);
                cur[j] = Math.min(sub, Math.min(prev[j] + 1, cur[j - 1] + 1));
            }
            int[] t = prev;
            prev = cur;
            cur = t;
        }
        return prev[b.length()];
    }

    // ---------------------------------------------------------------------------------------------
    // xml

    public String toXml() {
        return toXml(Collections.<String, Kind>emptyMap());
    }

    /** build the robot config XML (same lay as RC), throw if mistakes */
    public String toXml(Map<String, Kind> customTypes) {
        List<String> errs = new ArrayList<>();
        validate(customTypes, errs);
        if (!errs.isEmpty()) throw new ConfigException(errs);

        StringBuilder sb = new StringBuilder();
        sb.append("<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>\n");
        sb.append("<Robot type=\"FirstInspires-FTC\">\n");
        for (Object top : topLevel) {
            if (top instanceof Portal) {
                Portal p = (Portal) top;
                sb.append("    <LynxUsbDevice name=\"").append(esc(p.name)).append("\" serialNumber=\"")
                        .append(esc(p.serialNumber)).append("\" parentModuleAddress=\"").append(p.parentAddress).append("\">\n");
                for (Module<?> m : p.modules) {
                    sb.append("        <").append(m.xmlTag()).append(" name=\"").append(esc(m.name))
                            .append("\" port=\"").append(m.address).append("\">\n");
                    for (Device d : m.all()) {
                        sb.append("            <").append(d.type).append(" name=\"").append(esc(d.name))
                                .append("\" port=\"").append(d.port).append('"');
                        if (d.kind == Kind.I2C) sb.append(" bus=\"").append(d.bus).append('"');
                        sb.append(" />\n");
                    }
                    sb.append("        </").append(m.xmlTag()).append(">\n");
                }
                sb.append("    </LynxUsbDevice>\n");
            } else if (top instanceof Webcam) {
                Webcam w = (Webcam) top;
                sb.append("    <Webcam name=\"").append(esc(w.name)).append("\" serialNumber=\"")
                        .append(esc(w.serialNumber)).append("\" />\n");
            } else if (top instanceof EthernetDevice) {
                EthernetDevice e = (EthernetDevice) top;
                sb.append("    <EthernetDevice name=\"").append(esc(e.name)).append("\" serialNumber=\"")
                        .append(esc(e.serialNumber)).append("\" ipAddress=\"").append(esc(e.ipAddress)).append("\" />\n");
            }
        }
        sb.append("</Robot>\n");
        return sb.toString();
    }

    private static String esc(String s) {
        return s.replace("&", "&amp;").replace("\"", "&quot;").replace("<", "&lt;").replace(">", "&gt;");
    }

    // ---------------------------------------------------------------------------------------------
    // java (used by "Pull robot config")

    /** RobotConfig.HARDWARE expr */
    public String toJava(String indent) {
        Map<String, String> constants = new HashMap<>();
        for (Field f : Type.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers()) && f.getType() == String.class) {
                try {
                    String value = (String) f.get(null);
                    if (!constants.containsKey(value)) constants.put(value, f.getName());
                } catch (IllegalAccessException ignored) {
                }
            }
        }
        String i1 = indent + "        ";
        StringBuilder sb = new StringBuilder("new HardwareConfig(").append(str(name)).append(')');
        int usb = 0;
        for (Object top : topLevel) {
            if (top instanceof Portal) {
                Portal p = (Portal) top;
                if (p.embedded) {
                    for (Module<?> m : p.modules) sb.append('\n').append(i1).append(moduleJava(m, true, i1, constants));
                } else {
                    usb++;
                    String defaultName = "Expansion Hub Portal " + usb;
                    sb.append('\n').append(i1).append(".usbPortal(");
                    if (!p.name.equals(defaultName)) sb.append(str(p.name)).append(", ");
                    sb.append(str(p.serialNumber)).append(", ").append(p.parentAddress).append(", portal -> portal");
                    String i2 = i1 + "        ";
                    for (Module<?> m : p.modules) sb.append('\n').append(i2).append(moduleJava(m, false, i2, constants));
                    sb.append(')');
                }
            } else if (top instanceof Webcam) {
                Webcam w = (Webcam) top;
                sb.append('\n').append(i1).append(".webcam(").append(str(w.name)).append(", ").append(str(w.serialNumber)).append(')');
            } else if (top instanceof EthernetDevice) {
                EthernetDevice e = (EthernetDevice) top;
                sb.append('\n').append(i1).append(".ethernetDevice(").append(str(e.name)).append(", ")
                        .append(str(e.serialNumber)).append(", ").append(str(e.ipAddress)).append(')');
            }
        }
        return sb.toString();
    }

    private static String moduleJava(Module<?> m, boolean embeddedPortal, String indent, Map<String, String> constants) {
        StringBuilder sb = new StringBuilder();
        boolean servoHub = m instanceof ServoHub;
        if (embeddedPortal && !servoHub && m.address == CONTROL_HUB_ADDRESS && m.name.equals(CONTROL_HUB)) {
            sb.append(".controlHub(");
        } else if (!servoHub && m.name.equals("Expansion Hub " + m.address)) {
            sb.append(".expansionHub(").append(m.address).append(", ");
        } else if (servoHub && m.name.equals("Servo Hub " + m.address)) {
            sb.append(".servoHub(").append(m.address).append(", ");
        } else {
            sb.append(servoHub ? ".servoHub(" : ".hub(").append(str(m.name)).append(", ").append(m.address).append(", ");
        }
        List<Device> all = m.all();
        if (all.isEmpty()) return sb.append("hub -> {})").toString();
        sb.append("hub -> hub");
        String i = indent + "        ";
        for (Device d : all) sb.append('\n').append(i).append(deviceJava(d, constants));
        return sb.append(')').toString();
    }

    private static String deviceJava(Device d, Map<String, String> constants) {
        String n = str(d.name);
        String t = constants.containsKey(d.type) ? constants.get(d.type) : str(d.type);
        if (d.kind == null) return ".device(" + d.port + ", " + n + ", " + t + ")";
        switch (d.kind) {
            case MOTOR:
                return ".motor(" + d.port + ", " + n + ", " + t + ")";
            case SERVO:
                if (Type.SERVO.equals(d.type)) return ".servo(" + d.port + ", " + n + ")";
                if (Type.CR_SERVO.equals(d.type)) return ".crServo(" + d.port + ", " + n + ")";
                return ".servo(" + d.port + ", " + n + ", " + t + ")";
            case ANALOG:
                return Type.ANALOG_INPUT.equals(d.type) ? ".analog(" + d.port + ", " + n + ")" : ".analog(" + d.port + ", " + n + ", " + t + ")";
            case PWM:
                return Type.PWM_OUTPUT.equals(d.type) ? ".pwm(" + d.port + ", " + n + ")" : ".pwm(" + d.port + ", " + n + ", " + t + ")";
            case DIGITAL:
                return Type.DIGITAL_DEVICE.equals(d.type) ? ".digital(" + d.port + ", " + n + ")" : ".digital(" + d.port + ", " + n + ", " + t + ")";
            default:
                return ".i2c(" + d.bus + ", " + n + ", " + t + ")";
        }
    }

    private static String str(String s) {
        StringBuilder sb = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            if (c == '"' || c == '\\') sb.append('\\').append(c);
            else if (c == '\n') sb.append("\\n");
            else if (c < 0x20) sb.append(String.format("\\u%04x", (int) c));
            else sb.append(c);
        }
        return sb.append('"').toString();
    }

    static String join(List<String> parts, String sep) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < parts.size(); i++) {
            if (i > 0) sb.append(sep);
            sb.append(parts.get(i));
        }
        return sb.toString();
    }
}
