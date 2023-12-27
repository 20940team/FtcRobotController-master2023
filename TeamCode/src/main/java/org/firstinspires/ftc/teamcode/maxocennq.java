package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="333666999", group="")
public class maxocennq extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();

        DetectionPipelne pipeLine = new DetectionPipelne();
        pipeLine.targetColor = "RED";
        pipeLine.initFixed(telemetry);
        R.webcam.setPipeline(pipeLine);
        R.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                R.webcam.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard dash = FtcDashboard.getInstance();
                dash.startCameraStream(R.webcam, 30);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("result", pipeLine.getResult());
            R.delay(500);
        }

        //maxoceannq


        telemetry.addData("result", pipeLine.getResult());
        telemetry.update();

    }

}