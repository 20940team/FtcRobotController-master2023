package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="test povoroti", group="")
public class autotest2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();
        waitForStart();

       // R.LF.setPower(1);
        R.rotate(90);
        R.delay(10000);
        R.rotate(0);
        R.delay(10000);
    }
}