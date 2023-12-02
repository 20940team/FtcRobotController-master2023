package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot2023.close;
import static org.firstinspires.ftc.teamcode.Robot2023.open;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.Delayed;

@Autonomous(name="пиксель ну короче в клешней че то", group="")
public class plenk extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();

        waitForStart();

        R.grab.setPosition(close);
        R.setLift(-500);
        R.delay(1000);

    }
}