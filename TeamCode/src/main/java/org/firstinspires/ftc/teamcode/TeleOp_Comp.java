package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "3736: TeleOp", group = "TeleOp")

public class TeleOp_Comp extends OpMode {

    public static double MAX_SPEED = 1;
    public static double MIN_SPEED = 0.4;
    public static double MOTOR_SPEED = 1;
    public static boolean slow = true;
    SeriousHardware robot = new SeriousHardware();

	@Override
	public void init() {

        robot.init(hardwareMap);

        robot.DriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.PTORight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.DriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.PTOLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.PTORight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        robot.PTORight.setPower(y4 * MOTOR_SPEED);
        robot.PTOLeft.setPower(y4 * MOTOR_SPEED);
        robot.DriveLeft.setPower(y1 * MOTOR_SPEED);
        robot.DriveRight.setPower(y2 * MOTOR_SPEED);


		telemetry.addData("Text", "*** Robot Data***");
        telemetry.update();
    }



	@Override
	public void stop() {

	}

}



/*
im glad you have scrolled this far have a nice day!
yay




































 F you loser
 */