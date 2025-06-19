package com.protocols.magneticSwarmRec.gui;

import com.api.ArduSimTools;
import com.api.swarm.formations.Formation;
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

    // === Campos ===
    @FXML private TextField speed, threshold, minFlightDistance;
    @FXML private TextField frdOmni, frdDir, maxRepulsionFactor;
    @FXML private TextField omniRepulsionStrength, dirRepulsionStrength;
    @FXML private TextField dirFactor;
    @FXML private TextField weightAttraction, weightRepulsion;
    @FXML private TextField angleWeight, neighborDetectionRadius;

    @FXML private ComboBox<String> groundFormation, flyingFormation;

    public magneticSwarmRecDialogController(ResourceBundle resources, magneticSwarmRecSimProperties properties, Stage stage) {
        this.resources = resources;
        this.properties = properties;
        this.stage = stage;
    }

    @FXML
    public void initialize() {
        preloadFieldsFromProperties();

        for (Formation.Layout layout : Formation.Layout.values()) {
            groundFormation.getItems().add(layout.name());
            flyingFormation.getItems().add(layout.name());
        }

        groundFormation.setValue(magneticSwarmRecSimProperties.groundFormation.toUpperCase());
        flyingFormation.setValue(magneticSwarmRecSimProperties.flyingFormation.toUpperCase());

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
                    ComboBox<?> box = (ComboBox<?>) var.get(this);
                    if (box != null && box.getValue() != null) {
                        p.setProperty(name, box.getValue().toString());
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
            speed.setText(String.valueOf(magneticSwarmRecSimProperties.speed));
            threshold.setText(String.valueOf(magneticSwarmRecSimProperties.threshold));
            minFlightDistance.setText(String.valueOf(magneticSwarmRecSimProperties.minFlightDistance));
            frdOmni.setText(String.valueOf(magneticSwarmRecSimProperties.frdOmni));
            frdDir.setText(String.valueOf(magneticSwarmRecSimProperties.frdDir));
            maxRepulsionFactor.setText(String.valueOf(magneticSwarmRecSimProperties.maxRepulsionFactor));
            omniRepulsionStrength.setText(String.valueOf(magneticSwarmRecSimProperties.omniRepulsionStrength));
            dirRepulsionStrength.setText(String.valueOf(magneticSwarmRecSimProperties.dirRepulsionStrength));
            dirFactor.setText(String.valueOf(magneticSwarmRecSimProperties.dirFactor));
            weightAttraction.setText(String.valueOf(magneticSwarmRecSimProperties.weightAttraction));
            weightRepulsion.setText(String.valueOf(magneticSwarmRecSimProperties.weightRepulsion));
            angleWeight.setText(String.valueOf(magneticSwarmRecSimProperties.angleWeight));
            neighborDetectionRadius.setText(String.valueOf(magneticSwarmRecSimProperties.neighborDetectionRadius));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
