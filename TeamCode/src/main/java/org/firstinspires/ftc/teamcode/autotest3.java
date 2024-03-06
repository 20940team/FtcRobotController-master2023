package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 // отключен
@Autonomous(name="at", group="")
public class autotest3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();
        waitForStart();
        R.lift.close();
        R.lift2.close();
        R.grab1.setPower(1);
        R.delay(2500);
        R.grab1.setPower(0);
        R.delay(500);
        R.lift.setPosition(1);
        R.lift.setPosition(0);
        R.delay(10000);





    }
}