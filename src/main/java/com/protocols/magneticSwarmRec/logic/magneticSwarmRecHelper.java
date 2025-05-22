package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.ArduSimTools;
import com.api.ProtocolHelper;
import com.api.pojo.location.Waypoint;
import com.api.swarm.formations.Formation;
import com.api.swarm.formations.FormationFactory;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecDialogApp;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.setup.Text;
import com.setup.sim.logic.SimParam;
import es.upv.grc.mapper.*;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.javatuples.Pair;

import javax.swing.*;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.List;
import java.util.Properties;
import java.util.PropertyResourceBundle;
import java.util.ResourceBundle;

public class magneticSwarmRecHelper extends ProtocolHelper {

    @Override
    public void setProtocol() {
        this.protocolString = "MagneticSwarmReconfig";
    }

    @Override
    public boolean loadMission() {
        return true;
    }

    @Override
    public JDialog openConfigurationDialog() {
        return null;
    }

    @Override
    public void openConfigurationDialogFX() {
        // 🔄 Cargar los valores del archivo antes de abrir la GUI
        try (FileInputStream fis = new FileInputStream(SimParam.protocolParamFile)) {
            ResourceBundle resources = new PropertyResourceBundle(fis);
            Properties props = new Properties();
            for (String key : resources.keySet()) {
                props.setProperty(key, resources.getString(key));
            }
            magneticSwarmRecSimProperties propsLoader = new magneticSwarmRecSimProperties();
            propsLoader.storeParameters(props, resources);
        } catch (IOException e) {
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, "No se pudo cargar el archivo de parámetros.");
        }

        // 🖼 Mostrar interfaz de configuración
        Platform.runLater(() -> new magneticSwarmRecDialogApp().start(new Stage()));
    }

    @Override
    public void configurationCLI() {
        magneticSwarmRecSimProperties properties = new magneticSwarmRecSimProperties();
        try (FileInputStream fis = new FileInputStream(SimParam.protocolParamFile)) {
            ResourceBundle resources = new PropertyResourceBundle(fis);
            Properties p = new Properties();
            for (String key : resources.keySet()) {
                p.setProperty(key, resources.getString(key));
            }
            if (!properties.storeParameters(p, resources)) {
                ArduSimTools.warnGlobal(Text.LOADING_ERROR, "Error loading simulation parameters.");
                System.exit(1);
            }
        } catch (IOException e) {
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.PROTOCOL_PARAMETERS_FILE_NOT_FOUND);
            System.exit(0);
        }
    }

    @Override
    public void initializeDataStructures() {}

    @Override
    public String setInitialState() {
        return "WAITING";
    }

    @Override
    public Pair<Location2DGeo, Double>[] setStartingLocation() {
        int numUAVs = API.getArduSim().getNumUAVs();
        @SuppressWarnings("unchecked")
        Pair<Location2DGeo, Double>[] startingLocations = new Pair[numUAVs];

        if (magneticSwarmRecSimProperties.configMode.equalsIgnoreCase("FORMATION")) {
            Formation ground = FormationFactory.newFormation(
                    Formation.Layout.valueOf(magneticSwarmRecSimProperties.groundFormation.toUpperCase())
            );
            ground.init(numUAVs, magneticSwarmRecSimProperties.groundDistance);

            double centerLat = (magneticSwarmRecSimProperties.SWlat + magneticSwarmRecSimProperties.NElat) / 2.0;
            double centerLon = (magneticSwarmRecSimProperties.SWlon + magneticSwarmRecSimProperties.NElon) / 2.0;
            Location2DGeo centerGeo = new Location2DGeo(centerLat, centerLon);
            Location3DUTM centerUTM = new Location3DUTM(centerGeo.getUTM(), magneticSwarmRecSimProperties.altitude);

            for (int i = 0; i < numUAVs; i++) {
                try {
                    Location3DUTM pos = ground.get3DUTMLocation(centerUTM, i);
                    Location2DGeo geo = pos.getGeo3D();
                    startingLocations[i] = Pair.with(geo, 0.0);
                } catch (LocationNotReadyException e) {
                    e.printStackTrace();
                    ArduSimTools.warnGlobal("ERROR", "Failed to convert formation location to geo coordinates.");
                }
            }
        } else {
            // Modo manual: usar waypoint como posición inicial
            List<Waypoint>[] missions = API.getCopter(0).getMissionHelper().getMissionsLoaded();
            for (int i = 0; i < numUAVs; i++) {
                List<Waypoint> mission = missions[i];
                if (mission == null || mission.size() < 2) {
                    ArduSimTools.warnGlobal("ERROR", "No valid mission for UAV " + i);
                    continue;
                }
                Waypoint wp = mission.get(1);
                startingLocations[i] = Pair.with(new Location2DGeo(wp.getLatitude(), wp.getLongitude()), 0.0);
            }
        }

        return startingLocations;
    }

    @Override
    public boolean sendInitialConfiguration(int numUAV) {
        return true;
    }

    @Override
    public void startThreads() {}

    @Override
    public void setupActionPerformed() {}

    @Override
    public void startExperimentActionPerformed() {
        for (int i = 0; i < API.getArduSim().getNumUAVs(); i++) {
            new magneticSwarmAvoidance(i).start();
        }
    }

    @Override
    public void forceExperimentEnd() {}

    @Override
    public String getExperimentResults() {
        return null;
    }

    @Override
    public String getExperimentConfiguration() {
        return null;
    }

    @Override
    public void logData(String folder, String baseFileName, long baseNanoTime) {}

    @Override
    public void openPCCompanionDialog(JFrame PCCompanionFrame) {}
}
