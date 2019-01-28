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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.SeriousHardware.HANG_OPEN;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Crater Code", group = "Autonomous")

public class CraterCode extends LinearOpMode {

    SeriousHardware robot = new SeriousHardware();
    private ElapsedTime runtime = new ElapsedTime();

    double DLP;
    double DRP;

    public String Location;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

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
    private static final String VUFORIA_KEY = "ATSKisf/////AAAAGUc2Yh9CD0LGk3Dia2LEjAYdsSmUVQWL0upIBfSbNY7uuyqCuf1vkGSobE94Cl4K0J/X3Gq0qbvCuVQTGY/J12rvejNHaM0csIJbJjnFf+ceb1MDnoSCYXd8pdp0GBk8nR7STEj52Vr65Fposbvh+U0c8lp/pnVE5dHq3tih8tYmkp8fqPXlWqLX+i2ATyvv48rC0om5zpmRkXGNWCbh7mzgJpzYYKusPPYopBrLdpeJAxtGsmjii2I0Ub6AJ02pCLuYhOdnLDMUkiOgU6m1/3i2V8VSlUMoWusarb1EYhCoJ5xCoK8TIrSBvDpIFZBdrgueuFPWhmZSlitHTLRaGJZtsHvIPp5KstxzqQtdtwAf";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        robot.init(hardwareMap);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        robot.PTORight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Drop();

        Stop();

        robot.hang.setPosition(HANG_OPEN);

        sleep(500);




        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }


            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    Location = "Left";
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    Location = "Right";
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    Location = "Center";
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        runtime.reset();

        if (tfod != null) tfod.shutdown();

        sleep(2000);

        if (Location == "Right") {
            Turn(0.5, -0.5);
            sleep(500);
        } else if (Location == "Left") {
            Turn(0.5, -0.5);
            sleep(500);
        } else sleep(2000);

        GoForward(0.75);

        sleep(600);

        Stop();

        if (Location == "Right") {
            Turn(-0.5, 0.5);
            sleep(500);
        } else if (Location == "Left") {
            Turn(.5, -.5);
            sleep(500);
        } else sleep(500);

        GoForward(1);
        sleep(900);

        GoBackward(.2);
        sleep(100);

        ExtendArm(0.4);
        sleep(100);

        RotateArm(500, 0.4);
        sleep(2000);

        Stop();


        robot.PTORight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


        /**
         * Initialize the Vuforia localization engine.
        Stop();

        sleep(15000);

        }

     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    public void GoForward(double power) {
        robot.DriveLeft.setPower(power);
        robot.DriveRight.setPower(power);
    }

    public void GoBackward(double power) {
        GoForward(-power);
    }

    public void Drop() {
        robot.PTOLeft.setPower(1);
        robot.PTORight.setPower(1);
        sleep(3000);











































































































































    }

    public void Stop() {
        robot.PTORight.setPower(0);
        robot.PTOLeft.setPower(0);
        robot.DriveLeft.setPower(0);
        robot.DriveRight.setPower(0);
    }

    public void Turn(double DLP, double DRP) {

        robot.DriveLeft.setPower(DLP);
        robot.DriveRight.setPower(DRP);
    }

    public void RotateArm(int ticks, double power) {

        robot.RotateArm.setTargetPosition(ticks);
        robot.RotateArm.setPower(power);
    }

    public void ExtendArm(double power){
        robot.ExtendArmRight.setPower(-.75);
        robot.ExtendArmLeft.setPower(-.75);
    }

}

//* Thomas oh my god fix the f&*$#%@ code
//* Thomas and Logan you are doing a good job please keep trying you got this! :) also who ever threw that paper your mom is a hoe
  
