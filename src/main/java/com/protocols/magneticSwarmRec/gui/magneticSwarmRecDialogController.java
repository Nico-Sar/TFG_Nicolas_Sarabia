package com.protocols.magneticSwarmRec.gui;

import com.api.ArduSimTools;
import com.setup.Param;
import com.setup.Text;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import javafx.stage.Stage;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Properties;
import java.util.ResourceBundle;

public class magneticSwarmRecDialogController {

    private final ResourceBundle resources;
    private final Stage stage;
    private final magneticSwarmRecSimProperties properties;

    @FXML private Button okButton;
    @FXML private TextField altitude;
    @FXML private TextField frd;
    @FXML private TextField a;
    @FXML private TextField repulsionMagnitude;
    @FXML private TextField beaconingTime;
    @FXML private TextField SWlat;
    @FXML private TextField SWlon;
    @FXML private TextField NElat;
    @FXML private TextField NElon;
    @FXML private TextField minFlightDistance;
    @FXML private TextField seed;

    public magneticSwarmRecDialogController(ResourceBundle resources, magneticSwarmRecSimProperties properties, Stage stage) {
        this.resources = resources;
        this.properties = properties;
        this.stage = stage;
    }

    @FXML
    public void initialize(){
        okButton.setOnAction(e -> {
            if(ok()){
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
            String annotation = var.getAnnotatedType().getType().getTypeName();
            if (annotation.contains("javafx")) {
                try {
                    Method getValue = null;
                    if (annotation.contains("TextField")) {
                        getValue = var.get(this).getClass().getMethod("getCharacters");
                    }
                    if (getValue != null) {
                        String value = String.valueOf(getValue.invoke(var.get(this)));
                        p.setProperty(var.getName(), value);
                    }
                } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
                    e.printStackTrace();
                }
            }
        }
        return p;
    }
}
