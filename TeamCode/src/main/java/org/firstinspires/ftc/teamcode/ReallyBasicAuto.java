package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



/**
 * Created by 7t.qr on 12/13/2017.
 */

@Autonomous(name="Basic Blue", group ="Concept")

public class ReallyBasicAuto extends LinearOpMode {

    SeriousHardware robot  = new SeriousHardware();

    boolean Blue = true;
    boolean Red = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.hang.setPosition(0);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(2000);

        Drop();

        GoForward(0.75);

        sleep(2000);

        Stop();

        sleep(15000);
    }

    public void GoForward(double power) {
        robot.DriveLeft.setPower(power);
        robot.DriveRight.setPower(power);
    }

    public void GoBackward(double power) {
        GoForward(-power);
    }

    public void Drop() {
        robot.PTOLeft.setTargetPosition(-10);
        robot.PTORight.setTargetPosition(-10);

    }


    public void Stop() {
        robot.PTORight.setPower(0);
        robot.PTOLeft.setPower(0);
        robot.DriveLeft.setPower(0);
        robot.DriveRight.setPower(0);
    }

}
