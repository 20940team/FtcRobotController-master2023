package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.Delayed;
//@Disabled
@Autonomous(name="тест", group="")
public class autotest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();

        waitForStart();

        R.LF.setPower(1);
        R.delay(5000);
    }
}