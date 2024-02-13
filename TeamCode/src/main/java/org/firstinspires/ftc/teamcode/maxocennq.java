package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
@Autonomous(name="автоном по камере синий ", group="")
public class maxocennq extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();

        DetectionPipeLine pipeLine = new DetectionPipeLine();
        pipeLine.targetColor = "BLUE";
        pipeLine.initFixed(telemetry);
        R.webcam.setPipeline(pipeLine);

        R.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                R.webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard dash = FtcDashboard.getInstance();
                dash.startCameraStream(R.webcam, 30);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        boolean zonedetected = false;
        DetectionPipeLine.resultPosition result = pipeLine.getResult();
        while (!isStarted() && !isStopRequested()) {
            result = pipeLine.getResult();
            telemetry.addData("result", pipeLine.getResult());
            telemetry.update();
            R.delay(500);
        }

        //123 test staring now


        if (result == DetectionPipeLine.resultPosition.LEFT && zonedetected == false) {
            zonedetected = true;
            telemetry.addData("Going to the left zone! ", "1");
            telemetry.update();
            R.delay(1000);
            R.go(80);
            R.rotate(90);
            R.go(15);
            R.azaxvat(1000, -1);
            R.go(-25);

        } else if (result == DetectionPipeLine.resultPosition.CENTER && zonedetected == false) {
            zonedetected = true;
            telemetry.addData("Going to the center zone! ", "2");
            telemetry.update();
            R.delay(1000);
            R.go(95);
            R.azaxvat(1000, -1);
            R.go(-20);

        } else if (result == DetectionPipeLine.resultPosition.RIGHT && zonedetected == false) {
            zonedetected = true;
            telemetry.addData("Going to the right zone! ", "3");
            telemetry.update();
            R.delay(1000);
            R.go(80);
            R.rotate(-90);
            R.go(15);
            R.azaxvat(100, -1);
            R.go(-25);

        }


        telemetry.addData("result", pipeLine.getResult());
          R.delay(2000);
        telemetry.update();

    }

}