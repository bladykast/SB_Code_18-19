package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SeriousHardware;


import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.SeriousHardware.DUMP_CLOSED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.DUMP_OPEN;
import static org.firstinspires.ftc.teamcode.SeriousHardware.HANG_CLOSED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.HANG_OPEN;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MAX_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MIN_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.MOTOR_SPEED;
import static org.firstinspires.ftc.teamcode.SeriousHardware.PTO_Servo_Drive;
import static org.firstinspires.ftc.teamcode.SeriousHardware.PTO_Servo_Hang;
import static org.firstinspires.ftc.teamcode.SeriousHardware.ROTL_DOWN;
import static org.firstinspires.ftc.teamcode.SeriousHardware.ROTL_UP;
import static org.firstinspires.ftc.teamcode.SeriousHardware.ROTR_DOWN;
import static org.firstinspires.ftc.teamcode.SeriousHardware.ROTR_UP;


@TeleOp(name="3736 TeleOp", group="TeleOp")
public class TeleOp_Comp extends OpMode {

    //PTO Speeds
    public double PTOL_SPEED;
    public double PTOR_SPEED;

    double HangPos = HANG_CLOSED;
    double DumpPos = DUMP_CLOSED;
    String IntakePos = "Up";

    public boolean slow = true;
    public boolean reverse = false;

    boolean PTO_Hang;
    boolean PTO_Drive;

    public double armPosition;
    public int armPositionInt;

    public double armPositionDelta;

    boolean changedhang = false;
    boolean changeddump = false;
    boolean changedintake = false;


    SeriousHardware robot = new SeriousHardware();

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.DriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.PTORight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RotateArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ExtendArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ExtendArmRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.DriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.PTORight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RotateArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ExtendArmLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ExtendArmRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.RotateArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.PTOLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.PTORight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.hang.setPosition(HANG_CLOSED);
        robot.dump.setPosition(DUMP_CLOSED);
        robot.shifter.setPosition(PTO_Servo_Drive);

        PTO_Hang = true;
        PTO_Drive = false;


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


        //High-Low Speed Code
        if (slow) MOTOR_SPEED = MIN_SPEED;
        if (!slow) MOTOR_SPEED = MAX_SPEED;


        //Reverser Lander Code
        if (reverse){
            robot.DriveLeft.setPower(-y2 * MOTOR_SPEED);
            robot.DriveRight.setPower(-y1 * MOTOR_SPEED);
        }
        else {
            robot.DriveLeft.setPower(y1 * MOTOR_SPEED);
            robot.DriveRight.setPower(y2 * MOTOR_SPEED);
        }

        robot.PTOLeft.setPower(PTOL_SPEED);
        robot.PTORight.setPower(PTOR_SPEED);



        //Drivetrain Code
        if (PTO_Drive == true) {
            robot.shifter.setPosition(PTO_Servo_Drive);
            PTOL_SPEED = y1 * MOTOR_SPEED;
            PTOR_SPEED = y2 * MOTOR_SPEED;
        }
        else if (PTO_Drive == false) {
            robot.shifter.setPosition(PTO_Servo_Hang);
            PTOL_SPEED = (y3 * 0.6);
            PTOR_SPEED = (y3 * 0.6);
        }



        //Gamepad 1
        if ((reverse == true) && (gamepad1.y)) { reverse = false; }
        if ((reverse == false) && (gamepad1.b)) { reverse = true; }

        if ((slow == true) && (gamepad1.a)) { slow = false; }
        if ((slow == false) && (gamepad1.x)) { slow = true; }

        if (gamepad1.left_bumper) {
            robot.sweep.setPower(-1);
        } else if (gamepad1.right_bumper) {
            robot.sweep.setPower(1 );
        } else {
            robot.sweep.setPower(0);
        }

        if (gamepad1.dpad_down)
        { robot.ExtendArmLeft.setPower(0.5);
        robot.ExtendArmRight.setPower(0.5); }
        else if (gamepad1.dpad_up)
        { robot.ExtendArmLeft.setPower(-0.5);
        robot.ExtendArmRight.setPower(-0.5); }
        else
        { robot.ExtendArmLeft.setPower(0);
        robot.ExtendArmRight.setPower(0); }



        //Gamepad 2

        if(gamepad2.x && !changedhang) {
            if(HangPos == HANG_OPEN) HangPos = HANG_CLOSED;
            else HangPos = HANG_OPEN;
            changedhang = true;
        } else if(!gamepad2.x) changedhang = false;

        if(gamepad2.a && !changeddump) {
            if(DumpPos == DUMP_OPEN) DumpPos = DUMP_CLOSED;
            else DumpPos = DUMP_OPEN;
            changeddump = true;
        } else if(!gamepad2.a) changeddump = false;

        if(gamepad2.y && !changedintake) {
            if(IntakePos == "Up") IntakePos = "Down";
            else IntakePos = "Up";
            changedintake = true;
        } else if(!gamepad2.y) changedintake = false;

        if (IntakePos == "Down") {
            robot.rotl.setPosition(ROTL_UP);
            robot.rotr.setPosition(ROTR_UP);
        }
        if (IntakePos == "Up") {
            robot.rotl.setPosition(ROTL_DOWN);
            robot.rotr.setPosition(ROTR_DOWN);
        }



        //armPosition = Range.scale(gamepad2.right_trigger, 0, 1, 0, 5000);

        armPositionDelta = y4 * 8;
        armPosition += armPositionDelta;
        armPositionInt = (int) armPosition;

        robot.RotateArm.setTargetPosition(armPositionInt);
        robot.RotateArm.setPower(0.7);

        robot.hang.setPosition(HangPos);
        robot.dump.setPosition(DumpPos);

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Motor Speed", "Speed:  " + String.format("%.2f", MOTOR_SPEED));
        telemetry.addData("Actual Position", "Position:  " + String.format("%d", robot.RotateArm.getCurrentPosition()));
        telemetry.update();
    }


    @Override
    public void stop() {

    }

}
//* ok

