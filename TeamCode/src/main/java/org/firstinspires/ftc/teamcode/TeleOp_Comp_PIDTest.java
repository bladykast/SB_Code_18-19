package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Backbone.PIDController;

import static org.firstinspires.ftc.teamcode.SeriousHardware.ArmKD;
import static org.firstinspires.ftc.teamcode.SeriousHardware.ArmKI;
import static org.firstinspires.ftc.teamcode.SeriousHardware.ArmKP;
import static org.firstinspires.ftc.teamcode.SeriousHardware.HANG_OPEN;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MAX_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MIN_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.slow;

public class TeleOp_Comp_PIDTest extends OpMode {

    //Arm PID Controller Initialization
    PIDController ArmPID = new PIDController(ArmKP, ArmKI, ArmKD);

    //PTO Speeds
    public double PTOL_SPEED;
    public double PTOR_SPEED;

    SeriousHardware robot = new SeriousHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);



        robot.DriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.PTORight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.DriveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.PTORight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.hang.setPosition(HANG_OPEN);
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

        if (gamepad1.a) slow = true;
        if (gamepad1.x) slow = false;

        if (slow) MOTOR_SPEED = MIN_SPEED;
        if (!slow) MOTOR_SPEED = MAX_SPEED;

        //Drivetrain Code



        robot.DriveLeft.setPower(y1 * MOTOR_SPEED);
        robot.DriveRight.setPower(y2 * MOTOR_SPEED);
        robot.PTOLeft.setPower(PTOL_SPEED);
        robot.PTORight.setPower(PTOR_SPEED);


        //Gamepad 2

        final double error = (targetTicks - actualTicks);
        double motorPower = ArmPID.getPIDCorrection(error);
        if (motorPower > speed) motorPower = speed;

        robot.RotateArm.setPower(speed);


        //Gamepad 1


        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Motor Speed", "Speed:  " + String.format("%.2f", MOTOR_SPEED));
        telemetry.update();
    }


    @Override
    public void stop() {

    }

}