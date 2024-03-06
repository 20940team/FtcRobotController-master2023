package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "typ")
public class testteleop1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot2023 R = new Robot2023(hardwareMap, telemetry,this);
        R.init();
        R.gamepadInit(gamepad1, gamepad2);
        waitForStart();
        while (!isStopRequested()){
            R.testpomidor();
            R.telemetry();
        }
        telemetry.addData("Stop", "program");
        telemetry.update();

    }
}
