package com.protocols.magneticSwarmRec.gui;

import com.api.ArduSimTools;
import com.setup.Text;
import com.setup.sim.logic.SimParam;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.net.URL;
import java.util.PropertyResourceBundle;
import java.util.ResourceBundle;

public class magneticSwarmRecDialogApp extends Application {

    @Override
    public void start(Stage stage) {
        magneticSwarmRecSimProperties properties = new magneticSwarmRecSimProperties();
        ResourceBundle resources;

        // Carga el archivo de configuración del protocolo
        try (FileInputStream fis = new FileInputStream(SimParam.protocolParamFile)) {
            resources = new PropertyResourceBundle(fis);
        } catch (IOException e) {
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.PROTOCOL_PARAMETERS_FILE_NOT_FOUND);
            System.exit(1);
            return;
        }

        // Carga del archivo FXML
        FXMLLoader loader = new FXMLLoader();
        try {
            URL url = new File("src/main/resources/protocols/magneticSwarmRec/magneticSwarmRec.fxml").toURI().toURL();
            loader.setLocation(url);
        } catch (IOException e) {
            e.printStackTrace();
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.ERROR_LOADING_FXML);
            System.exit(1);
            return;
        }

        magneticSwarmRecDialogController controller = new magneticSwarmRecDialogController(resources, properties, stage);
        loader.setController(controller);
        loader.setResources(resources);

        stage.setTitle("Magnetic Swarm Reconfiguration");

        try {
            stage.setScene(new Scene(loader.load()));
        } catch (IOException e) {
            e.printStackTrace();
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.ERROR_LOADING_FXML);
            System.exit(1);
        }

        stage.setOnCloseRequest(event -> System.exit(0));
        stage.show();
    }
}
