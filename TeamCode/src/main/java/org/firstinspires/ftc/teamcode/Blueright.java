/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Blue right", group = "Zhou")
//@Disabled
public class Blueright extends LinearOpMode {

    HardwareMecanum3 robot   = new HardwareMecanum3();   // Use the 3rd hardware
    private ElapsedTime runtime = new ElapsedTime();
    private int position = -1; //default to cup not seen/left position

    /* Note: This sample uses the all-objects Tensor Flow model (cup.tflite), which contains
     * the following 1 detectable object
     *  0: Cup
     */
    private static final String TFOD_MODEL_ASSET = "cup.tflite";
    private static final String[] LABELS = {
            "Cup",
    };

    static final double     H_SPEED = 0.8;
    static final double     M_SPEED = 0.5;
    static final double     S_SPEED = 0.3;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AYpGLkP/////AAABmVjkrwtpeEdGh5mFOy38cLslB/Q5w/O9y8O8x+w3KBRwPcXKtqzdME32ANUZQPPkmaO648RxsAx9OnPOLnL2VEjhkjcB+AG9PexkDlapER41ioHTyyO0N1sBPZqnmyJHhS23Yxb7hzY3ArVtRNavso4ZBoB2EgfQjBQrIbQrzS96eW+CLZYh31pcoHtoKHH5Fm435clFOLnbnXGcLIUZjB1khsJSqmSoLP5tjHnd8WC6lkzcYxZFVemMzDr7YHa444GDe+Sy04sppeiL1IL75Foqrt2aUNg/yTlFXoEmFKHdeiGawnF5t+DChDhsxRlfOQDadXkKe3G1ysHgDlSTeuySd0Ol6d1AjJilSrpIQq7B";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * Variables used for switching cameras.
     */
    private WebcamName webcam1, webcam2;
    private SwitchableCamera switchableCamera;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

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
            // (typically 16/9).
            tfod.setZoom(1, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //set motors on encoder
            robot.reset();
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // 1. Find cup
            position = findCup();
            sleep(500);
            /* Needs adjusting */
            // 4. Put the preloaded cube in based on the cup's position
            double forwardInchValue = 0;
            if (position==0) { //left
                robot.liftUp(1,0,1);
                forwardInchValue = 1;
            }
            else if (position==1) { //middle
                robot.liftUp(1,6,3);
                forwardInchValue=2;
            }
            else { //right
                robot.liftUp(1,10.7,4);
                forwardInchValue=3;
            }
            // 2. Once the lift is lifted, navigate to the depot thing
            sleep(500);
            robot.rightInches(M_SPEED,36.5, 3);
            robot.forwardInches(S_SPEED,forwardInchValue,3);
            sleep(1500);

            //3. Next, push cube out
            robot.rollerSpeed(-1,1);
            sleep(500);

            //4. Back to the wall
            robot.forwardInches(S_SPEED,-forwardInchValue,3);
            robot.rightInches(M_SPEED,-31.5,5);
            sleep(1000);

            //5. Do Red Spin
            robot.forwardInches(M_SPEED,-32,5); //approach carousel
            robot.forwardInches(S_SPEED, -1,1); //ensure contact
            robot.spinnerL.setPower(1); //spin
            runtime.reset();
            while (opModeIsActive() && runtime.seconds()<5) {
                telemetry.addData("spinning", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            robot.spinnerL.setPower(0);
            robot.rightInches(M_SPEED,20,5);    //go to storage
            robot.forwardInches(S_SPEED,-10,3);  //back to wall



            stop();
        }
    }

    public int findCup() {
        int location = -1;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                    // Change "Duck" to "Cup"
                    if (recognition.getLabel() == "Cup") {
                        if (((recognition.getLeft() + recognition.getRight()) / 2) < 200) {
                            location = 0;
                            telemetry.addData("left", location);
                        } else if (((recognition.getLeft() + recognition.getRight()) / 2) > 300) {
                            location = 1;
                            telemetry.addData("middle", location);
                        }
                    }
                    else {
                            location = -1;
                            telemetry.addData("right", 0);
                        }
                }
                telemetry.update();
            }
        }
        return location;
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
        // Indicate that we wish to be able to switch cameras.
        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Set the active camera to Webcam 2.
        switchableCamera = (SwitchableCamera) vuforia.getCamera();
        switchableCamera.setActiveCamera(webcam2);

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
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}
