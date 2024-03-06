package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="автоном по камере красный", group="")
public class autoredcam extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this); // инициализация полей
        R.init(); // инициализация камеры и направления моторов

        DetectionPipeLine pipeLine = new DetectionPipeLine(); // создавние нового пайплайна типа detection pipeline
        pipeLine.targetColor = "RED"; // задание искомого цвета
        pipeLine.initFixed(telemetry); // подключение телметрии и полей
        R.webcam.setPipeline(pipeLine); // присвоение камере пайплайна

        R.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                R.webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT); // задание напрвления разрешения камеры
                FtcDashboard dash = FtcDashboard.getInstance(); // подключение камеры к дешборду
                dash.startCameraStream(R.webcam, 30); // ограничение фпс
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        DetectionPipeLine.resultPosition result = pipeLine.getResult(); // присвоение переменной результата
        while (!isStarted() && !isStopRequested()) {
            result = pipeLine.getResult(); // обновление результата
            telemetry.addData("result", pipeLine.getResult());
            telemetry.update();
            R.delay(500);
        }

        //123 test staring now


        if (result == DetectionPipeLine.resultPosition.LEFT) {  // проверка результата с соответствущими зонами
            telemetry.addData("Going to the left zone! ", "1");
            telemetry.update();
            R.delay(1000);
            R.ngo(22);
            R.delay(1000);
            R.rotate(60);
            R.delay(1500);
            R.ngo(-5);

        } else if (result == DetectionPipeLine.resultPosition.CENTER) {
            telemetry.addData("Going to the center zone! ", "2");
            telemetry.update();
            R.delay(1000);
            R.ngo(23);
            R.delay(1000);
            R.ngo(-10);
        } else if (result == DetectionPipeLine.resultPosition.RIGHT) {
            telemetry.addData("Going to the right zone! ", "3");
            telemetry.update();
            R.delay(1000);
            R.ngo(23);
            R.delay(1000);
            R.rotate(-90);
            R.delay(1500);
            R.ngo(10);
            R.delay(1000);
            R.ngo(-10);

        }


        telemetry.addData("result", pipeLine.getResult());
          R.delay(2000);
        telemetry.update();

    }

}