package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 // отключен
@Config
@Autonomous(name="jestb avtonom bez kamera testing", group="")
public class autotest1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();

       // R.goTimer(0,0.8, 2200);
        R.delay(500);

        waitForStart();
        R.ngo(20);
        R.delay(100000);


    }
}
/* center
        R.delay(1000);
        R.ngo(23.5);
        R.delay(1000);
        R.ngo(-10);



     right
        R.ngo(23);
        R.delay(1000);
        R.rotate(-90);
        R.delay(1500);
        R.ngo(7.5);
        R.delay(1000);
        R.ngo(-10);


     left
        R.ngo(22);
        R.delay(1000);
        R.rotate(90);
        R.delay(1500);
        R.ngo(-5);
        */