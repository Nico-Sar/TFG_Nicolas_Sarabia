package com.protocols.magneticSwarmRec.gui;

import com.api.API;
import com.api.ArduSimTools;
import com.api.swarm.formations.Formation;
import com.protocols.magneticSwarmRec.gui.magneticSwarmRecSimProperties;
import com.setup.Param;
import com.setup.Text;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.stage.Stage;

import java.lang.reflect.Field;
import java.util.Properties;
import java.util.ResourceBundle;

public class magneticSwarmRecDialogController {

    private final ResourceBundle resources;
    private final Stage stage;
    private final magneticSwarmRecSimProperties properties;

    @FXML private Button okButton;

    @FXML private TextField altitude, speed, frd, alpha, dirFactor, dirRatio, tDist;
    @FXML private TextField weightAttraction, weightRepulsion, minFlightDistance, beaconingTime, seed;
    @FXML private TextField repulsionMagnitude;

    @FXML private ComboBox<String> configMode, groundFormation, flyingFormation;

    public magneticSwarmRecDialogController(ResourceBundle resources, magneticSwarmRecSimProperties properties, Stage stage) {
        this.resources = resources;
        this.properties = properties;
        this.stage = stage;
    }

    @FXML
    public void initialize() {
        preloadFieldsFromProperties();

        configMode.getItems().addAll("Formation", "Single");
        configMode.setValue(magneticSwarmRecSimProperties.configMode.toLowerCase());

        for (Formation.Layout layout : Formation.Layout.values()) {
            groundFormation.getItems().add(layout.name());
            flyingFormation.getItems().add(layout.name());
        }

        groundFormation.setValue(magneticSwarmRecSimProperties.groundFormation.toUpperCase());
        flyingFormation.setValue(magneticSwarmRecSimProperties.flyingFormation.toUpperCase());

        // Activar/desactivar campos de formación según el modo
        configMode.setOnAction(e -> {
            boolean isFormation = "formation".equalsIgnoreCase(configMode.getValue());
            groundFormation.setDisable(!isFormation);
            flyingFormation.setDisable(!isFormation);
            if (!isFormation) {
                groundFormation.setValue("");
                flyingFormation.setValue("");
            }
        });


        okButton.setOnAction(e -> {
            if (ok()) {
                Platform.setImplicitExit(false);
                Param.simStatus = Param.SimulatorState.STARTING_UAVS;
                okButton.getScene().getWindow().hide();
            } else {
                ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.ERROR_LOADING_FXML);
            }
        });
    }

    private boolean ok() {
        Properties p = createProperties();
        return properties.storeParameters(p, resources);
    }

    private Properties createProperties() {
        Properties p = new Properties();
        Field[] variables = this.getClass().getDeclaredFields();

        for (Field var : variables) {
            try {
                var.setAccessible(true);
                String name = var.getName();
                String type = var.getType().getSimpleName();

                if (type.equals("TextField")) {
                    TextField field = (TextField) var.get(this);
                    if (field != null && !field.isDisabled()) {
                        String value = field.getText().trim();
                        if (!value.isEmpty()) {
                            p.setProperty(name, value);
                        }
                    }
                } else if (type.equals("ComboBox")) {
                    ComboBox<String> box = (ComboBox<String>) var.get(this);
                    if (box != null && box.getValue() != null && !box.getValue().isEmpty()) {
                        p.setProperty(name, box.getValue());
                    }
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        return p;
    }

    private void preloadFieldsFromProperties() {
        try {
            altitude.setText(String.valueOf(magneticSwarmRecSimProperties.altitude));
            speed.setText(String.valueOf(magneticSwarmRecSimProperties.speed));
            frd.setText(String.valueOf(magneticSwarmRecSimProperties.frd));
            alpha.setText(String.valueOf(magneticSwarmRecSimProperties.alpha));
            dirFactor.setText(String.valueOf(magneticSwarmRecSimProperties.dirFactor));
            dirRatio.setText(String.valueOf(magneticSwarmRecSimProperties.dirRatio));
            tDist.setText(String.valueOf(magneticSwarmRecSimProperties.tDist));
            weightAttraction.setText(String.valueOf(magneticSwarmRecSimProperties.weightAttraction));
            weightRepulsion.setText(String.valueOf(magneticSwarmRecSimProperties.weightRepulsion));
            minFlightDistance.setText(String.valueOf(magneticSwarmRecSimProperties.minFlightDistance));
            beaconingTime.setText(String.valueOf(magneticSwarmRecSimProperties.beaconingTime));
            seed.setText(String.valueOf(magneticSwarmRecSimProperties.seed));
            repulsionMagnitude.setText(magneticSwarmRecSimProperties.repulsionMagnitude);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
