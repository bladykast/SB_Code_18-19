package org.firstinspires.ftc.teamcode.Backbone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

@TeleOp
public class SamplingOpMode extends OpMode {
    //Direction is relative to the front of the robot immediately after landing.
    public enum SampleOrder{
        LEFT,
        RIGHT,
        CENTER,
        UNKNOWN
    }
    SampleOrderDetector cubeDetector;
    SampleOrder Order;
    @Override
    public void init(){
        Order = SampleOrder.UNKNOWN;
        cubeDetector = new SampleOrderDetector();
        //0 should be rear camera, 1 should be front camera
        cubeDetector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(),0);
        cubeDetector.enable();
        //cubeDetector.onCameraViewStarted(1920,1080);
    }
    @Override
    public void loop(){
        //TODO: TUNE THE THRESHOLDS (THE INTEGERS IN THE NESTED IF STATEMENTS) WHEN THE PHONE IS ACTUALLY MOUNTED TO THE ROBOT. ROUGH RANGES SEEM TO WORK WITH EYEBALLING
        if(cubeDetector.CubePosY != 0 && cubeDetector.CubePosX != 0){
            if(cubeDetector.CubePosY > 180){
                Order = SampleOrder.LEFT;
            }
            else if (cubeDetector.CubePosY < 175 && cubeDetector.CubePosY > 100){
                Order = SampleOrder.CENTER;
            }
            else if (cubeDetector.CubePosY < 100){
                Order = SampleOrder.RIGHT;
            }
            else{
                Order = SampleOrder.UNKNOWN;
            }
        }
        else {
            Order = SampleOrder.UNKNOWN;
        }
        telemetry.addData("Current Order", Order);
        telemetry.addData("CubePosX", cubeDetector.CubePosX);
        telemetry.addData("CubePosY", cubeDetector.CubePosY);
        telemetry.update();
    }
    @Override
    public void stop(){
        cubeDetector.disable();
    }
}
