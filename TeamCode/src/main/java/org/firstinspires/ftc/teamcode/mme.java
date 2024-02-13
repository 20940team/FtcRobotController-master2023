package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="nevkluchaite test", group="")
public class mme extends LinearOpMode {
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

        double subresult = 0;
        DetectionPipeLine.resultPosition result = pipeLine.getResult();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("result", pipeLine.getResult());
            R.delay(500);
        }

        //123 test staring now

        if (result == DetectionPipeLine.resultPosition.LEFT) {
            telemetry.addData("Going to the left zone! ","1");
            subresult = 1;

        } else if (result == DetectionPipeLine.resultPosition.CENTER) {
            telemetry.addData("Going to the center zone! ","2");
            subresult = 2;

        } else if (result == DetectionPipeLine.resultPosition.RIGHT) {
            telemetry.addData("Going to the right zone! ","3");
            subresult = 3;

        }


        telemetry.addData("result", pipeLine.getResult());
        R.delay(2000);
        telemetry.update();

    }

}