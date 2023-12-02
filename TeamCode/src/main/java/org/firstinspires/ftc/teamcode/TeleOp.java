package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp1")

public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry,this);
        R.init();
        R.gamepadInit(gamepad1, gamepad2);
        waitForStart();
        while (!isStopRequested()){
            R.driveOmni();
            R.teleOp();
            R.zaxvat();
            //R.initLift();
            R.telemetry();
        }
        telemetry.addData("Stop", "program");
        telemetry.update();

    }
}
