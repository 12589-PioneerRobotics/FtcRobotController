/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.core;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.concurrent.Future;
import java.util.concurrent.FutureTask;

import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.core.FieldConstants.leftOfRingPos;

public class TensorFlowRingDetection implements Runnable {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AUpVsRb/////AAABmfXBUZLrt08+nrSaM7v/vidtLHZ0UdsN9r8vpRV8EuuFUiiyGWgGVuci/lZTSRA39PzVEGsoM8A5BcsrpPIXk8hdkPH3N2rlibpM36nPkCm+uzLNSSDSjZGwTlXO9L03O86py1p5d1sAKCZ2x9W0djHYvK7EbbmUhRzxhlXpUeud9TetcF/+7QZTxm3eIGN6y0N16Ifd/1F0hPTglegVIegbdrSYnLo8uvRkdHr0LncFus50LcZh7iq/d9jyKKqVmdVuIAUHh5FBTUC3w4QQyWD54wABK4aNO62I013+Bts24EFk/ZQpoHlNMgXl0MyuvkS0BEZQENBf8MR4vON0UgADxZNfcwmG+eZNeTB2jX8w";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    LinearOpMode linearOpMode;
    public String ringCase = "";
    public boolean stopFlag = false;

    public Future<Trajectory> getInitialPath() {
        switch (ringCase) {
            case "None":
                return pathA;
            case LABEL_SECOND_ELEMENT:
                return pathB;
            case LABEL_FIRST_ELEMENT:
                return pathC;
        }
        return null;
    }

    Future<Trajectory> initialPath, pathA, pathB, pathC;

    StandardMechanumDrive drive;


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public TensorFlowRingDetection(LinearOpMode linearOpMode, StandardMechanumDrive drive) {
        this.drive = drive;
        this.linearOpMode = linearOpMode;
        initVuforia();
        initTfod(linearOpMode.hardwareMap);
        if (tfod != null) {
            tfod.activate();
        }

        /*pathA = pathB = drive.trajectory(
                () -> drive.trajectoryBuilder(
                        drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(0, -30), 0)
                        .build());
        pathC = drive.trajectory(() -> drive.trajectoryBuilder(drive.getPoseEstimate(), toRadians(45))
                .splineToConstantHeading(leftOfRingPos, toRadians(225))
                .splineToConstantHeading(new Vector2d(-2, -30), toRadians(135))
                .build());*/
    }

    @Override
    public void run() {
        Log.v("CV Thread", "begin");
        while (!stopFlag) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
//                  linearOpMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (!updatedRecognitions.isEmpty())
                        ringCase = updatedRecognitions.get(0).getLabel();
                    else ringCase = "None";

                    linearOpMode.telemetry.addData("Ring case", ringCase);
                    linearOpMode.telemetry.update();
                }
            }
            if (stopFlag && tfod != null) {
                tfod.shutdown();
                return;
            }
        }


        if (tfod != null) {
            tfod.shutdown();
        }
    }

}
