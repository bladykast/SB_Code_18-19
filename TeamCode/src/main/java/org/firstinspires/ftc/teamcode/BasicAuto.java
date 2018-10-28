/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.SeriousHardware.glyleftDown;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyleftUp;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyphdumpUp;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyrightDown;
import static org.firstinspires.ftc.teamcode.SeriousHardware.glyrightUp;

/**
 This is the 2017-2018 Relic Recovery Code, integrating Drive-By-Encoder, the BNO055 IMU, an Adafruit RGB Sensor, and Vuforia.
 */

@Autonomous(name="Red Audience Side", group ="Concept")
public class BasicAuto extends LinearOpMode {

    // Define if Red or Blue side, and if audience or ref side.
    boolean Red = true;
    boolean Blue = false;

    boolean Audience = true;
    boolean Ref = false;

    /* Declare OpMode members. */
    SeriousHardware         robot   = new SeriousHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    BNO055IMU imu;
    AdafruitI2cColorSensor sensorRGB;

    // Variables for BNO055
    Orientation angles;
    Acceleration gravity;

    double heading;

    // Encoder Variables
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .667 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // PID Constants
    static final double     DRIVE_SPEED             = 0.4;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.4;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    //Vuforia
    OpenGLMatrix lastLocation = null;
    String LCR;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        //Initialize Robot
        robot.init(hardwareMap);

        //Initialize BNO055 and Calibrate
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters BNOparameters = new BNO055IMU.Parameters();
        BNOparameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        BNOparameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        BNOparameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        BNOparameters.loggingEnabled = true;
        BNOparameters.loggingTag = "IMU";
        BNOparameters.useExternalCrystal = true;
        BNOparameters.mode = BNO055IMU.SensorMode.IMU;
        BNOparameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(BNOparameters);

        //Stop and Reset Encoders
        StopEncodersAndWait();



        /** Wait for the game to begin */
        telemetry.addData(">", "Calibrating Gyro");
        telemetry.update();

        while (!isStarted()) {
            UpdateHeading();

            telemetry.addData(">", "Robot Ready.");
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.addData("Heading = ", "%.2f", heading);
            telemetry.update();
        }
        telemetry.update();
        waitForStart();

        runtime.reset();

        //Drop Jewel arm
        robot.jewel.setPosition(0.65);
        sleep(1000);


    }


    public void PIDDrive ( double speed,
                            double distance, double angle) {

        int     newLeftFrontTarget;
        int     newRightRearTarget;
        int     newRightFrontTarget;
        int     newLeftRearTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftSideBack.getCurrentPosition() + moveCounts;
            newLeftFrontTarget = robot.leftSideFront.getCurrentPosition() + moveCounts;
            newRightRearTarget = robot.rightSideBack.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightSideFront.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftSideBack.setTargetPosition(newLeftRearTarget);
            robot.rightSideBack.setTargetPosition(newRightRearTarget);
            robot.leftSideFront.setTargetPosition(newLeftFrontTarget);
            robot.rightSideFront.setTargetPosition(newRightFrontTarget);

            robot.leftSideBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSideBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftSideFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightSideFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftSideBack.setPower(speed);
            robot.rightSideBack.setPower(speed);
            robot.leftSideFront.setPower(speed);
            robot.rightSideFront.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftSideBack.isBusy() && robot.rightSideBack.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftSideBack.setPower(leftSpeed);
                robot.rightSideBack.setPower(rightSpeed);
                robot.leftSideFront.setPower(leftSpeed);
                robot.rightSideFront.setPower(rightSpeed);
            }

            // Stop all motion;
            robot.leftSideBack.setPower(0);
            robot.rightSideBack.setPower(0);
            robot.leftSideFront.setPower(0);
            robot.rightSideFront.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftSideBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSideBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftSideFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightSideFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.;
            UpdateHeading();
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            UpdateHeading();
            telemetry.update();
        }

        // Stop all motion;
        robot.leftSideBack.setPower(0);
        robot.rightSideBack.setPower(0);
        robot.rightSideFront.setPower(0);
        robot.leftSideFront.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftSideBack.setPower(leftSpeed);
        robot.rightSideBack.setPower(rightSpeed);
        robot.leftSideFront.setPower(leftSpeed);
        robot.rightSideFront.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - heading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    double formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    double formatDegrees(double degrees) {
        return AngleUnit.DEGREES.normalize(degrees);
    }

    void UpdateHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = (angles.firstAngle+360)%360;
    }

    public void GoForward(double power) {
        robot.rightSideBack.setPower(power);
        robot.leftSideBack.setPower(power);
        robot.rightSideFront.setPower(power);
        robot.leftSideFront.setPower(power);
    }

    public void GoBackward(double power) {
        GoForward(-power);
    }

    public void Stop() {
        robot.rightSideBack.setPower(0);
        robot.leftSideBack.setPower(0);
        robot.rightSideFront.setPower(0);
        robot.leftSideFront.setPower(0);
        robot.strafe.setPower(0);
    }

    public void StopEncodersAndWait(){
        robot.rightSideBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSideBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightSideFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftSideFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.strafe.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);

        robot.leftSideBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightSideBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightSideFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftSideFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.strafe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Strafe(double power) {
        robot.strafe.setPower(power);
    }
}
