package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.SeriousHardware.ArmKD;
import static org.firstinspires.ftc.teamcode.SeriousHardware.ArmKI;
import static org.firstinspires.ftc.teamcode.SeriousHardware.ArmKP;
import static org.firstinspires.ftc.teamcode.SeriousHardware.HANG_OPEN;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MAX_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MIN_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.PTO_Servo_Drive;
import static org.firstinspires.ftc.teamcode.SeriousHardware.PTO_Servo_Hang;

public class TeleOp_Comp extends OpMode {

    //PTO Speeds
    public double PTOL_SPEED;
    public double PTOR_SPEED;

    public boolean slow = true;

    boolean PTO_Hang;
    boolean PTO_Drive;

    // Encoder Variables
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = .682 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    public double armPosition;
    public int armPositionInt;

    SeriousHardware robot = new SeriousHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.DriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.PTORight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.DriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.PTORight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RotateArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.RotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.hang.setPosition(HANG_OPEN);
        robot.shifter.setPosition(PTO_Servo_Drive);
    }

    public void loop()
    {
        /*
         *  Note from 2017: Yes, forward on the stick is f$#@ing -1. We have had all of our robots
         *  driving backwards for the last 3 years.
         *
         *  Lovely, huh?
         */

        float y1 = -gamepad1.left_stick_y;
        float y2 = -gamepad1.right_stick_y;
        float y3 = -gamepad2.left_stick_y;
        float y4 = -gamepad2.right_stick_y;
        boolean b1 = gamepad2.left_bumper;
        boolean b2 = gamepad2.right_bumper;
        boolean dleft = gamepad1.dpad_left;
        boolean dright = gamepad1.dpad_right;

        y1 = Range.clip(y1, -1, 1);
        y2 = Range.clip(y2, -1, 1);
        y3 = Range.clip(y3, -1, 1);
        y4 = Range.clip(y4, -1, 1);


        //High-Low Speed Code
        if ((slow == true) && (gamepad1.x)) {
            slow = false;
        }
        if ((slow == false) && (gamepad1.x)) {
            slow = true;
        }
        if (slow) MOTOR_SPEED = MIN_SPEED;
        if (!slow) MOTOR_SPEED = MAX_SPEED;


        //Drivetrain Code
        PTO_Hang = !PTO_Drive;

        robot.DriveLeft.setPower(y1 * MOTOR_SPEED);
        robot.DriveRight.setPower(y2 * MOTOR_SPEED);
        robot.PTOLeft.setPower(PTOL_SPEED);
        robot.PTORight.setPower(PTOR_SPEED);

        if (PTO_Drive = true) {
            robot.shifter.setPosition(PTO_Servo_Drive);
            PTOL_SPEED = y1 * MOTOR_SPEED;
            PTOR_SPEED = y2 * MOTOR_SPEED;
    }
        else if (PTO_Hang = true) {
            robot.shifter.setPosition(PTO_Servo_Hang);
            PTOL_SPEED = y3;
            PTOR_SPEED = y3;
        }


        //Gamepad 1


        //Gamepad 2
        armPosition = Range.scale(gamepad2.right_trigger, 0, 1, 0, 5000);
        armPositionInt = (int) armPosition;
        robot.RotateArm.setTargetPosition(armPositionInt);


        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Motor Speed", "Speed:  " + String.format("%.2f", MOTOR_SPEED));
        telemetry.addData("Arm Position", "Position:" + String.format("%.2f", robot.RotateArm.getCurrentPosition()));
        telemetry.update();
    }


    @Override
    public void stop() {

    }

}