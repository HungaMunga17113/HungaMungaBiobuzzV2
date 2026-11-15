package org.firstinspires.ftc.teamcode.util.hardware;

import org.w3c.dom.Element;
import org.w3c.dom.NamedNodeMap;
import org.w3c.dom.Node;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.xml.parsers.DocumentBuilderFactory;

/**
 * ran by "Pull robot config";
 * robot config xml -> java for RobotConfig
 * usage:
 * ConfigImporter &lt;xml&gt; &lt;config name&gt; &lt;indent&gt; &lt;output file&gt; [KIND=xmlTag ...]
 */
public final class ConfigImporter {
    private final Map<String, HardwareConfig.Kind> custom;
    private final List<String> warnings = new ArrayList<>();

    ConfigImporter(Map<String, HardwareConfig.Kind> custom) {
        this.custom = custom;
    }

    public static void main(String[] args) throws Exception {
        File xml = new File(args[0]);
        String configName = args[1];
        String indent = args[2];
        File out = new File(args[3]);
        ConfigImporter importer = new ConfigImporter(ConfigWriter.customTypes(args, 4));

        HardwareConfig config = importer.read(xml, configName);
        for (String w : importer.warnings) System.out.println("warning: " + w);

        List<String> errors = new ArrayList<>();
        for (String w : config.validate(importer.custom, errors)) System.out.println("warning: " + w);
        for (String e : errors) System.out.println("warning: the config on the robot has a problem, Push will refuse it until fixed: " + e);

        try (Writer w = new OutputStreamWriter(new FileOutputStream(out), StandardCharsets.UTF_8)) {
            w.write(config.toJava(indent));
        }
    }

    HardwareConfig read(File file, String configName) throws Exception {
        DocumentBuilderFactory factory = DocumentBuilderFactory.newInstance();
        Element root = factory.newDocumentBuilder().parse(file).getDocumentElement();
        if (!root.getTagName().equals("Robot")) throw new IllegalArgumentException(file.getName() + " is not a robot configuration");

        HardwareConfig config = new HardwareConfig(configName);
        for (Element top : children(root)) {
            String tag = top.getTagName();
            if (tag.equals("LynxUsbDevice")) {
                readPortal(config, top);
            } else if (tag.equals("Webcam")) {
                extraAttributes(top, "name", "serialNumber");
                config.webcam(top.getAttribute("name"), top.getAttribute("serialNumber"));
            } else if (tag.equals("EthernetDevice")) {
                extraAttributes(top, "name", "serialNumber", "ipAddress", "port");
                config.ethernetDevice(top.getAttribute("name"), top.getAttribute("serialNumber"), top.getAttribute("ipAddress"));
            } else {
                warnings.add("skipping <" + tag + "> (the RC ignores it too)");
            }
        }
        return config;
    }

    private void readPortal(HardwareConfig config, Element portal) {
        extraAttributes(portal, "name", "serialNumber", "parentModuleAddress");
        String serial = portal.getAttribute("serialNumber");
        if (serial.equals(HardwareConfig.EMBEDDED_SERIAL)) {
            int parent = intAttr(portal, "parentModuleAddress", HardwareConfig.CONTROL_HUB_ADDRESS);
            if (!portal.getAttribute("name").equals(HardwareConfig.CONTROL_HUB_PORTAL)) {
                warnings.add("Control Hub portal is named \"" + portal.getAttribute("name") + "\", it'll be renamed \"" + HardwareConfig.CONTROL_HUB_PORTAL + "\"");
            }
            for (final Element module : children(portal)) {
                final int address = intAttr(module, "port", -1);
                final String name = module.getAttribute("name");
                if (module.getTagName().equals("LynxModule")) {
                    // = control hub
                    if (address == parent || address == HardwareConfig.CONTROL_HUB_ADDRESS) {
                        if (name.equals("Control Hub")) config.controlHub(hub -> readHub(hub, module));
                        else config.hub(name, HardwareConfig.CONTROL_HUB_ADDRESS, hub -> readHub(hub, module));
                    } else {
                        config.hub(name, address, hub -> readHub(hub, module));
                    }
                } else if (module.getTagName().equals("ServoHub")) {
                    config.servoHub(name, address, hub -> readServoHub(hub, module));
                } else {
                    warnings.add("skipping <" + module.getTagName() + "> in the Control Hub portal");
                }
            }
        } else {
            int parent = intAttr(portal, "parentModuleAddress", 1);
            config.usbPortal(portal.getAttribute("name"), serial, parent, p -> {
                for (final Element module : children(portal)) {
                    int address = intAttr(module, "port", -1);
                    String name = module.getAttribute("name");
                    if (module.getTagName().equals("LynxModule")) p.hub(name, address, hub -> readHub(hub, module));
                    else if (module.getTagName().equals("ServoHub")) p.servoHub(name, address, hub -> readServoHub(hub, module));
                    else warnings.add("skipping <" + module.getTagName() + "> in " + portal.getAttribute("name"));
                }
            });
        }
    }

    private void readHub(HardwareConfig.Hub hub, Element module) {
        extraAttributes(module, "name", "port");
        List<Element> i2c = new ArrayList<>();
        for (Element d : children(module)) {
            String type = d.getTagName();
            String name = d.getAttribute("name");
            int port = intAttr(d, "port", -1);
            HardwareConfig.Kind kind = kindOf(type);
            if (kind == HardwareConfig.Kind.I2C || (kind == null && d.hasAttribute("bus"))) {
                i2c.add(d);
                continue;
            }
            extraAttributes(d, "name", "port");
            if (kind == null) {
                warnings.add(type + " (\"" + name + "\") is an unknown device type, imported with device()");
                hub.device(port, name, type);
                continue;
            }
            switch (kind) {
                case MOTOR: hub.motor(port, name, type); break;
                case SERVO: hub.servo(port, name, type); break;
                case ANALOG: hub.analog(port, name, type); break;
                case PWM: hub.pwm(port, name, type); break;
                default: hub.digital(port, name, type); break;
            }
        }
        // i2c poses on bus
        Collections.sort(i2c, new Comparator<Element>() {
            @Override public int compare(Element a, Element b) {
                int bus = Integer.compare(bus(a), bus(b));
                return bus != 0 ? bus : Integer.compare(intAttr(a, "port", 0), intAttr(b, "port", 0));
            }
        });
        for (Element d : i2c) {
            extraAttributes(d, "name", "port", "bus");
            hub.i2c(bus(d), d.getAttribute("name"), d.getTagName());
        }
    }

    private void readServoHub(HardwareConfig.ServoHub hub, Element module) {
        extraAttributes(module, "name", "port");
        for (Element d : children(module)) {
            extraAttributes(d, "name", "port");
            hub.servo(intAttr(d, "port", -1), d.getAttribute("name"), d.getTagName());
        }
    }

    private HardwareConfig.Kind kindOf(String type) {
        HardwareConfig.Kind kind = HardwareConfig.sdkKind(type);
        return kind != null ? kind : custom.get(type);
    }

    /** legacy configs dont have bus attr so sdk uses port as bus */
    private static int bus(Element d) {
        return d.hasAttribute("bus") ? intAttr(d, "bus", 0) : intAttr(d, "port", 0);
    }

    private void extraAttributes(Element e, String... known) {
        Set<String> ok = new HashSet<>(Arrays.asList(known));
        NamedNodeMap attrs = e.getAttributes();
        for (int i = 0; i < attrs.getLength(); i++) {
            String a = attrs.item(i).getNodeName();
            if (!ok.contains(a)) {
                warnings.add("dropping " + a + "=\"" + attrs.item(i).getNodeValue() + "\" from <" + e.getTagName()
                        + " name=\"" + e.getAttribute("name") + "\"> (not supported)");
            }
        }
    }

    private static int intAttr(Element e, String attr, int fallback) {
        String v = e.getAttribute(attr);
        if (v.isEmpty()) return fallback;
        try {
            return Integer.parseInt(v.trim());
        } catch (NumberFormatException ex) {
            throw new IllegalArgumentException("<" + e.getTagName() + " name=\"" + e.getAttribute("name") + "\"> has " + attr + "=\"" + v + "\", expected a number");
        }
    }

    private static List<Element> children(Element e) {
        List<Element> out = new ArrayList<>();
        for (Node n = e.getFirstChild(); n != null; n = n.getNextSibling()) {
            if (n instanceof Element) out.add((Element) n);
        }
        return out;
    }
}
