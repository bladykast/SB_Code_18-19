

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Outreach Bot TeleOp", group = "TeleOp")
public class OutreachTeleOp extends OpMode {

    DcMotor strafe;
    DcMotor rightSide;
    DcMotor leftSide;

    public OutreachTeleOp() {

    }

    @Override
    public void init() {

        strafe = hardwareMap.dcMotor.get("STR");
        rightSide = hardwareMap.dcMotor.get("MR");
        leftSide = hardwareMap.dcMotor.get("ML");
        rightSide.setDirection(DcMotor.Direction.REVERSE);
        leftSide.setDirection(DcMotor.Direction.FORWARD);
        strafe.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop()
    {
        float y1 = -gamepad1.left_stick_y;
        float y2 = -gamepad1.right_stick_y;
        float y3 = -gamepad2.left_stick_y;
        float y4 = -gamepad2.right_stick_y;
        boolean dleft = gamepad1.dpad_left;
        boolean dright = gamepad1.dpad_right;

        leftSide.setPower(y2);
        rightSide.setPower(y1);

        if (gamepad1.dpad_left)
        {
            strafe.setPower(-1);
        }
        else if (gamepad1.dpad_right)
        {
            strafe.setPower(1);
        }
        else strafe.setPower(0);


        telemetry.addData("Text", "*** Robot Data***");
    }


    @Override
    public void stop() { }
}