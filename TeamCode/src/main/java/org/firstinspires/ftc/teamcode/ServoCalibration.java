package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// отключен
@Config
@Autonomous(name="servo calibration", group="")
public class ServoCalibration extends LinearOpMode {
    public static double ServoPos = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();
        R.gamepadInit(gamepad1, gamepad2);
        R.delay(500);
        waitForStart();
        while (!isStopRequested()) {
            if (gamepad1.left_bumper) {
                R.samolet.setPosition(ServoPos);
                ServoPos += 0.01;
                R.delay(200);
            } else if (gamepad1.right_bumper) {
                R.samolet.setPosition(ServoPos);
                ServoPos -= 0.01;
                R.delay(200);
            }
            telemetry.addData("servopos", ServoPos);
            telemetry.update();
        }



    }

}