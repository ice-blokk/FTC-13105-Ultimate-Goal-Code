package org.firstinspires.ftc.teamcode.RobotClasses.Subsystems;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class ObjectDetection {
    // Tensorflow and Vuforia
    private static WebcamName webcam = null;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite", LABEL_FIRST_ELEMENT = "Quad", LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AXSMEWz/////AAABmXindJ41FU7dqka7VWsVrrdl63f0OizJlhSQ0ne6I+tFB3+LMayyYDcS5+1+oc/dm5LQTwqw/sbnUxm4SNJAK/8qNrQXAmwdUns8AVa5RVkHLDIn69qalfO7GCFqo/cfOAJTEpCAms6O9oK20qHpwa/SJ9GOpdQDPu71+tVuJzXm3ZJTy4zSIfn9yxse5+K8Hhbi8QuoIFmlUEmJLrrwoEGYR02cp/oihTGufFvEsplZTq5uGtdwn9Q5O7vW+b/+Rk1B6W0W/kLobli7tB0TZpJli3pw4KzOoD6vuIyS2nRcmM221bmeBia/YCf+4/xACnPYzs3pQC2DVA05ls9MB7DFPit2Z1QqIUxae2SHRFLq";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Recognition trueRec;
    private int tfodID;

    public ObjectDetection(WebcamName webcam, int tfodMonitorViewId) {
        this.webcam = webcam;
        this.tfodID = tfodMonitorViewId;
    }

    // Cases:
    // null - nothing
    // trueRec.getLabel().equals("Single") - single stack
    // else - quad

    public void initVision() {
        // Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcam; //hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        //TensorFlow
        //tfodID = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodID);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        tfod.activate();
    }

    public Recognition seeRing() {
        Recognition rec = null;
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                double maxArea = Integer.MIN_VALUE;
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getHeight() * recognition.getWidth() > maxArea) {
                        rec = recognition;
                        maxArea = rec.getHeight() * rec.getWidth();
                    }
                }
            }
        }

        return rec;
    }

}// end of class


