package com.protocols.magneticSwarmRec.logic;

import com.api.API;
import com.api.ArduSimTools;
import com.api.ProtocolHelper;
import com.api.pojo.location.Waypoint;
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
import java.util.*;


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
        Platform.runLater(() -> new magneticSwarmRecDialogApp().start(new Stage()));
    }

    @Override
    public void configurationCLI() {
        magneticSwarmRecSimProperties properties = new magneticSwarmRecSimProperties();
        ResourceBundle resources;
        try {
            FileInputStream fis = new FileInputStream(SimParam.protocolParamFile);
            resources = new PropertyResourceBundle(fis);
            fis.close();
            Properties p = new Properties();
            for (String key : resources.keySet()) {
                p.setProperty(key, resources.getString(key));
            }
            properties.storeParameters(p, resources);
        } catch (IOException e) {
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.PROTOCOL_PARAMETERS_FILE_NOT_FOUND);
            System.exit(0);
        }
    }

    @Override
    public void initializeDataStructures() {
        // Inicialización de variables globales del protocolo si es necesario
    }

    @Override
    public String setInitialState() {
        return "WAITING";
    }

    @Override
    public Pair<Location2DGeo, Double>[] setStartingLocation() {
        int numUAVs = API.getArduSim().getNumUAVs();
        @SuppressWarnings("unchecked")
        Pair<Location2DGeo, Double>[] startingLocations = new Pair[numUAVs];

        List<Waypoint>[] missions = API.getCopter(0).getMissionHelper().getMissionsLoaded();

        for (int i = 0; i < numUAVs; i++) {
            List<Waypoint> uavMission = missions[i];
            if (uavMission == null || uavMission.size() < 2) {
                API.getGUI(0).exit("ERROR: UAV " + i + " has invalid mission data.");
            }
            Waypoint wp = uavMission.get(1);  // Primer punto válido después del dummy
            startingLocations[i] = Pair.with(new Location2DGeo(wp.getLatitude(), wp.getLongitude()), 0.0);
        }

        return startingLocations;
    }


    @Override
    public boolean sendInitialConfiguration(int numUAV) {
        return true;
    }

    @Override
    public void startThreads() {
        // Puede usarse para inicializar estructuras si es necesario antes del experimento
    }

    @Override
    public void setupActionPerformed() {
        // Coordinación inicial si es necesaria antes del experimento
    }

    @Override
    public void startExperimentActionPerformed() {
        int numUAVs = API.getArduSim().getNumUAVs();
        for (int i = 0; i < numUAVs; i++) {
            magneticSwarmAvoidance thread = new magneticSwarmAvoidance(i);
            thread.start();
        }
    }

    @Override
    public void forceExperimentEnd() {
        // Lógica para forzar el fin si se requiere
    }

    @Override
    public String getExperimentResults() {
        return null;
    }

    @Override
    public String getExperimentConfiguration() {
        return null;
    }

    @Override
    public void logData(String folder, String baseFileName, long baseNanoTime) {
        // Registrar datos de trayectoria, métricas u otros parámetros
    }

    @Override
    public void openPCCompanionDialog(JFrame PCCompanionFrame) {
        // No implementado por ahora
    }

}
