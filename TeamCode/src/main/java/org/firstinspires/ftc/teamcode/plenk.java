package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot2023.close;
import static org.firstinspires.ftc.teamcode.Robot2023.open;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.Delayed;
@Autonomous(name="dt", group="")
public class plenk extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry, this);
        R.init();

        waitForStart();
        R.setMtAll(1);
        R.delay(2000);
        R.setMtZero();



    }
}