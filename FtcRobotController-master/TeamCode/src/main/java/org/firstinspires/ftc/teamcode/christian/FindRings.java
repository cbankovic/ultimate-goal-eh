package org.firstinspires.ftc.teamcode.christian;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.SwitchData;

import java.util.List;

@Disabled
@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")
public class FindRings extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AXbaM4X/////AAABmasKHKdK60IXjDtokRJeu6w/6UeS3KpD2vdyQEvji2tDLEy+IToNeCC4oU0iEAOEzsgw8FABI0qMvJ001KiAfvt3YQETOyjFMY++rqydjE49FLsEPSMGzch3gzelSF9gw3gusCb0rhP/GGXaZnYpN4HbYYI9o/7jUgenQTxlblDFwDsjSgf8TiIoJGTMsW77RCv90nhsWlD+i8qYEUwM3pCxlQ0jImn1+uTTQfoLRNJEn1ZCrDaTcjf5+yxsgdHDXyB5Xh9hd031YFVjX8nX+m9n1ZDHAp8Ha3nLH1MYM5TUuh2/CNZMgyw2BPpAaasW4hT9aDiaYKAVlHQ32dVlTIie2Za4gVFGHgHaahZyUMuz";


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
//            tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {

                            float right = recognition.getBottom();
                            float left = recognition.getTop();
                            float top = recognition.getRight();
                            float bottom = recognition.getLeft();

                            float height = Math.abs( top - bottom);
                            float width = Math.abs(right - left);
                            float area = Math.abs(height * width);
                            float xCenter = (right + left) / 2;
                            float yCenter = (top + bottom) / 2;

                            // 1280 x 720 = 720p Resolution
                            // 640 (x) x 360 (y) = center
                            String horizontal = (xCenter < 620  ? "Left" : (xCenter > 660 ? "Right" : "Good"));
                            String vertical = (yCenter < 350  ? "Down" : (yCenter > 370 ? "Up" : "Good"));

                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  top, left (%d)", i), "%.03f , %.03f",
                                    top, left);
                            telemetry.addData(String.format("  bottom, right (%d)", i), "%.03f , %.03f",
                                    bottom , right);
                            telemetry.addData(String.format("  height, width (%d)", i), "%.03f , %.03f",
                                    height, width);
                            telemetry.addData(String.format("  area (%d)", i), "%.03f ",area);
                            telemetry.addData(String.format("  xC, yC (%d)", i), "%.03f , %.03f",
                                    xCenter, yCenter);
                            telemetry.addData(String.format("  horizon, vert (%d)", i), "%s, %s", horizontal, vertical);


                            if (area <= 24000){
                                telemetry.addData("Distance", "14 inches");
                            } else if (area <= 42000){
                                telemetry.addData("Distance", "12 inches");
                            } else if (area <= 59000){
                                telemetry.addData("Distance", "10 inches");
                            } else if (area <= 99000){
                                telemetry.addData("Distance", "8 inches");
                            } else if (area <= 105000){
                                telemetry.addData("Distance", "6 inches");
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
