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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 */

public class SeriousHardware
{

    //Declarations
    public DcMotor DriveRight, PTOLeft, DriveLeft, PTORight, RotateArm, ExtendArm, Intake = null;
    public Servo shifter, hang = null;
    public BNO055IMU imu = null;
    public DigitalChannel digitalTouch = null;


    /*  Variables: all constants are tweaked here  */

    //Virtual Hi-Low Speeds
    public static double MAX_SPEED = 1;
    public static double MIN_SPEED = 0.4;
    public static double MOTOR_SPEED = 1;

    //Servo Positions
    public static double HANG_CLOSED = 0;
    public static double HANG_OPEN = 1;
    public static double PTO_Servo_Hang = 1;
    public static double PTO_Servo_Drive = 1;

    //Arm PID Variables
    public static double ArmKP = 1;
    public static double ArmKI = 1;
    public static double ArmKD = 1;




    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public SeriousHardware() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Motors
        DriveLeft = hwMap.get(DcMotor.class, "DL");
        DriveRight = hwMap.get(DcMotor.class, "DR");
        PTOLeft = hwMap.get(DcMotor.class, "PTOL");
        PTORight = hwMap.get(DcMotor.class, "PTOR");
        RotateArm = hwMap.get(DcMotor.class, "ARMROT");
        ExtendArm = hwMap.get(DcMotor.class, "ARMEXT");
        Intake = hwMap.get(DcMotor.class, "SWEEP");

        //Servos, CR Servos & EDR 393s
        hang = hwMap.get(Servo.class, "HNG");

        //Sensors
        digitalTouch = hwMap.get(DigitalChannel.class, "Touch");
        imu = (BNO055IMU) hwMap.get("imu");


        //Motor Configuration
        DriveLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveRight.setDirection(DcMotor.Direction.REVERSE);
        PTOLeft.setDirection(DcMotor.Direction.REVERSE);
        PTORight.setDirection(DcMotor.Direction.FORWARD);

        DriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PTOLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PTORight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Servo Positions
        hang.setPosition(0);


    }
}

