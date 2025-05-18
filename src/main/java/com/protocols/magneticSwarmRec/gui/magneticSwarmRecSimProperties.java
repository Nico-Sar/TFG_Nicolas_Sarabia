package com.protocols.magneticSwarmRec.gui;

import com.api.API;
import com.api.ArduSimTools;
import com.api.MissionHelper;
import com.api.pojo.location.Waypoint;
import com.setup.Text;
import es.upv.grc.mapper.Location2DGeo;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.util.EnumValue;
import org.javatuples.Pair;

import java.io.File;
import java.lang.reflect.Field;
import java.util.*;

public class magneticSwarmRecSimProperties {

    public static double SWlat;
    public static double SWlon;
    public static double NElat;
    public static double NElon;
    public static double minFlightDistance;
    public static double altitude = 40;
    public static double a;
    public static String repulsionMagnitude;
    public static boolean randomPath;
    public static List<File> missionFile;
    public static int seed;
    public static int beaconingTime;
    public static double frd = 30;         // Full Repulsion Distance
    public static double alpha = 20;       // Ancho de la parábola gamma
    public static double dirFactor = 2;    // Control del ángulo central
    public static double dirRatio = 0.3;   // Repulsión base (omnidireccional)

    private Random random;

    public boolean storeParameters(Properties guiParams, ResourceBundle fileParams) {
        Properties parameters = new Properties();
        for (String key : fileParams.keySet()) {
            parameters.setProperty(key, guiParams.containsKey(key) ? guiParams.getProperty(key) : fileParams.getString(key));
        }

        Field[] variables = this.getClass().getDeclaredFields();
        Map<String, Field> variablesDict = new HashMap<>();
        for (Field var : variables) {
            variablesDict.put(var.getName(), var);
        }

        for (Object keyObj : parameters.keySet()) {
            String key = keyObj.toString();
            String value = parameters.getProperty(key);
            if (!variablesDict.containsKey(key) || value == null || value.trim().isEmpty()) continue;
            Field var = variablesDict.get(key);
            try {
                String type = var.getType().toString();
                if (type.equals("int")) {
                    var.setInt(this, Integer.parseInt(value.trim()));
                } else if (type.equals("double")) {
                    var.setDouble(this, Double.parseDouble(value.trim()));

                } else if (type.contains("java.lang.String")) {
                    var.set(this, value);
                } else if (type.contains("boolean")) {
                    var.set(this, Boolean.parseBoolean(value));
                } else if (type.contains("java.util.List")) {
                    String[] filesNames = value.split(";");
                    List<File> files = new ArrayList<>();
                    for (String fileName : filesNames) {
                        fileName = API.getFileTools().getResourceFolder().toString() + File.separator + fileName;
                        File f = new File(fileName);
                        String extension = fileName.substring(fileName.lastIndexOf('.') + 1);
                        if (f.exists() && (extension.equals(Text.FILE_EXTENSION_WAYPOINTS) || extension.equals(Text.FILE_EXTENSION_KML))) {
                            files.add(f);
                        }
                    }
                    var.set(this, files);
                } else {
                    ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.ERROR_STORE_PARAMETERS + type);
                    return false;
                }
            } catch (IllegalAccessException e) {
                return false;
            }
        }

        String error = specificCheckVariables();
        if (error == null) {
            setSimulationParameters();
            return true;
        } else {
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, "Error in parameter: " + error);
            return false;
        }
    }

    private String specificCheckVariables() {
        if (!randomPath) {
            if (missionFile == null || missionFile.size() == 0) return "missionFile is zero";
            for (File f : missionFile) {
                if (!f.exists()) return "missionFile does not exist";
            }
        }
        return null;
    }

    private void setSimulationParameters() {
        storeMissionFile();
    }

    private void storeMissionFile() {
        Pair<String, List<Waypoint>[]> missions = randomPath ? setMissionWaypoints() : API.getGUI(0).loadMissions(missionFile);
        MissionHelper missionHelper = API.getCopter(0).getMissionHelper();
        missionHelper.setMissionsLoaded(missions.getValue1());
        API.getArduSim().setNumUAVs(Math.min(missions.getValue1().length, API.getArduSim().getNumUAVs()));
    }

    private Pair<String, List<Waypoint>[]> setMissionWaypoints() {
        random = new Random(seed);
        int numUAVs = API.getArduSim().getNumUAVs();
        MissionHelper missionHelper = API.getCopter(0).getMissionHelper();
        EnumValue<MavCmd> cmd_waypoint = EnumValue.of(MavCmd.MAV_CMD_NAV_WAYPOINT);
        EnumValue<MavCmd> cmd_takeoff = EnumValue.of(MavCmd.MAV_CMD_NAV_TAKEOFF);
        EnumValue<MavCmd> cmd_land = EnumValue.of(MavCmd.MAV_CMD_NAV_LAND);

        Waypoint start = new Waypoint(0, true, MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT, cmd_waypoint,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1);

        List<Waypoint>[] al = new ArrayList[numUAVs];
        List<Location2DGeo> initialPositions = new ArrayList<>();
        for (int i = 0; i < al.length; i++) {
            al[i] = new ArrayList<>();
            al[i].add(start);

            boolean farEnough = true;
            Location2DGeo l1 = null;

            while (l1 == null) {
                l1 = getRandomLocation();
                for (Location2DGeo ip : initialPositions) {
                    if (l1.getUTM().distance(ip.getUTM()) < 15) {
                        farEnough = false;
                    }
                }
                if (farEnough) {
                    initialPositions.add(l1);
                } else {
                    l1 = null;
                    farEnough = true;
                }
            }

            Location2DGeo l2 = null;
            while (l2 == null) {
                l2 = getRandomLocation();
                if (l1.getUTM().distance(l2.getUTM()) < minFlightDistance) {
                    l2 = null;
                }
            }

            Waypoint takeoff = new Waypoint(1, false, MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT, cmd_takeoff,
                    0.0, 0.0, 0.0, 0.0,
                    l1.latitude, l1.longitude, altitude, 1);
            al[i].add(takeoff);

            Waypoint w1 = new Waypoint(2, false, MavFrame.MAV_FRAME_GLOBAL, cmd_waypoint,
                    0.0, 0.0, 0.0, 0.0,
                    l2.latitude, l2.longitude, altitude, 1);
            al[i].add(w1);

            Waypoint land = new Waypoint(3, false, MavFrame.MAV_FRAME_GLOBAL, cmd_land,
                    0.0, 0.0, 0.0, 0.0,
                    l2.latitude, l2.longitude, 30.0, 0);
            al[i].add(land);
        }
        missionHelper.setMissionsLoaded(al);
        return new Pair<>("generatedPath", al);
    }

    private Location2DGeo getRandomLocation() {
        double lat = SWlat + (NElat - SWlat) * random.nextDouble();
        double lon = SWlon + (NElon - SWlon) * random.nextDouble();
        return new Location2DGeo(lat, lon);
    }
}
