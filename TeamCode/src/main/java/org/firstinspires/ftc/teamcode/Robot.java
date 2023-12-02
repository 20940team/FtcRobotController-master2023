package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Robot {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;
    LinearOpMode linearOpMode;


    Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.linearOpMode = linearOpMode;
    }


    public void gamepadInit(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }


    public final void delay(double milliseconds) {
        try {
            Thread.sleep((long) milliseconds);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

}
//20940megapassword