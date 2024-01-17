package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
public class Robot2023 extends Robot {

    DcMotor RF, LF, LB , AA, RB, UP1, UP2;
    Servo grab, lift, lift2, samolet;
    BNO055IMU imu;

    public static double open = 0.2;
    public static double close = 0.7;
    public static double lopen = 0;
    public static double lclose = 1;
    public static double addservolift = 0.85;
    public static double dregee = 0;

    public static double kr = 0.05;

    Robot2023(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) {
        super(hardwareMap, telemetry, linearOpMode);
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        UP1 = hardwareMap.get(DcMotor.class, "UP1");
        UP2 = hardwareMap.get(DcMotor.class, "UP2");
        AA = hardwareMap.get(DcMotor.class, "AA");
        grab = hardwareMap.get(Servo.class, "grab");
        lift2 = hardwareMap.get(Servo.class, "lift2");
        lift = hardwareMap.get(Servo.class, "lift");
        samolet = hardwareMap.get(Servo.class, "samolet");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //Акселерометр
        parameters.angleUnit           = DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) { //Калибровка акселерометра
            delay(30);
            telemetry.addData("Wait", "Calibration"); //Сообщение о калибровке
            telemetry.update();
        }
        telemetry.addData("Done!", "Calibrated"); //Сообщение об окончании калибровки
        telemetry.update();
    }
    public double startTick;

    double getAngle() { //Функция получения данных с акселерометра
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration gravity = imu.getGravity();
        return angles.firstAngle;
    }

    public void driveOmni(){
        double lf = (gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.left_trigger*0.5) - (gamepad1.right_trigger*0.5));
        double lb = (gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.left_trigger*0.5) - (gamepad1.right_trigger*0.5));
        double rf = (gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.left_trigger*0.5) + (gamepad1.right_trigger*0.5));
        double rb = (gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.left_trigger*0.5) + (gamepad1.right_trigger*0.5));
        LF.setPower(lf); // - - + + left trigger right trigger
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }

    long map(long x, long in_min, long in_max, long out_min, long out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public void drive() {
        double lf = (gamepad1.left_stick_y + gamepad1.right_trigger*0.5 - gamepad1.left_trigger*0.5);
        double lb = (gamepad1.left_stick_y + gamepad1.right_trigger*0.5 - gamepad1.left_trigger*0.5);
        double rf = (gamepad1.left_stick_y - gamepad1.right_trigger*0.5 + gamepad1.left_trigger*0.5);
        double rb = (gamepad1.left_stick_y - gamepad1.right_trigger*0.5 + gamepad1.left_trigger*0.5);
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);


    }
    public void telemetry() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("left_y", gamepad1.left_stick_y);
        dashboardTelemetry.addData("left_x", gamepad1.left_stick_x);
        dashboardTelemetry.addData("right trigger", gamepad1.right_trigger);
        dashboardTelemetry.addData("left trigger", gamepad1.left_trigger );
        dashboardTelemetry.addData("salomet", samolet.getPosition());
        dashboardTelemetry.addData("lift position", lift.getPosition());
        dashboardTelemetry.addData("box position", grab.getPosition());
        dashboardTelemetry.addData("power up1+", UP1.getPower());
        dashboardTelemetry.addData("power up2-", UP2.getPower());
        dashboardTelemetry.addData("ticks lf", LF.getCurrentPosition());
        dashboardTelemetry.addData("power LF", LF.getPower());
        dashboardTelemetry.addData("power LB", LB.getPower());
        dashboardTelemetry.addData("power RF", RF.getPower());
        dashboardTelemetry.addData("power RB", RB.getPower());
        dashboardTelemetry.addData("angle", getAngle());
        dashboardTelemetry.update();
    }

    public void init() {
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        initCamera();
    }


    public void teleOp() {

        UP1.setPower(((gamepad2.right_stick_y)*(gamepad2.right_stick_y)*Math.signum(gamepad2.right_stick_y)));
        UP2.setPower(-((gamepad2.right_stick_y)*(gamepad2.right_stick_y)*Math.signum(gamepad2.right_stick_y)));
        // if (startTick < UP.getCurrentPosition()) {
        // UP.setPower(-0.1);}



        while (gamepad2.y) AA.setPower(1);
        AA.setPower(0);


        if (gamepad2.x) {
            grab.setPosition(close);
        } else if (gamepad2.a) {
            grab.setPosition(open);}

        if (gamepad2.dpad_left) {
            samolet.setPosition(1);
        } else if (gamepad2.dpad_right) {
            samolet.setPosition(0);}

        if (gamepad2.dpad_up) {
            lift.setPosition(lopen);
            lift2.setPosition(1-lopen);
        } else if (gamepad2.dpad_down) {
            lift.setPosition(lclose);
            lift2.setPosition(1-lclose);
        } else if (gamepad2.b) {
            lift.setPosition(addservolift);
            lift2.setPosition(addservolift);
        }

        telemetry.addData("right trigger: ", gamepad1.right_trigger);
        telemetry.addData("left trigger: ", gamepad1.left_trigger );
        // telemetry.addData("encoder", startTick);
        telemetry.addData("getAngle: ", getAngle());
        telemetry.addData("grab angles", grab.getPosition());
        telemetry.update();
    }



    public void arm(double x, double time) {
        UP1.setPower(x);
        UP2.setPower(-x);
        delay(time);
        UP1.setPower(-0);
        UP2.setPower(0);
    }

    public void azaxvat(double time, double direction) {
        AA.setPower(1*Math.signum(direction));
        delay(time);
        AA.setPower(0);
    }

    public void goTimer(double x, double y, double time) {
        LF.setPower(y - x);
        LB.setPower(y + x);
        RF.setPower(y + x);
        RB.setPower(y - x);
        delay(time);
        setMtZero();
    }

    public void setMtPower(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }

    public void setMtZero() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }
    public void setMtAllDelay(double power, double time) {
        LF.setPower(power);
        LB.setPower(power);
        RF.setPower(power);
        RB.setPower(power);
        delay(time);
        setMtZero();
    }

    public void setMtAll(double power) {
        LF.setPower(power);
        LB.setPower(power);
        RF.setPower(power);
        RB.setPower(power);
    }

    public void servoClose() {
        grab.setPosition(close);
    }

    public void servoOpen() {
        grab.setPosition(open);
    }


    public void testpomidor() {
        double error = 2;

        double kp = 0.085;
        while (error>0.1 && linearOpMode.opModeIsActive()) {
            error = -getAngle();
            double p = kp * error;
            double rele = kr * Math.signum(error);
            double pwf = p + rele;
            setMtPower(pwf, pwf, -pwf, -pwf);
        }
    }
    public void gotocoord(double xpos, double ypos) {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ccx = (1450 * xpos) / (9.5 * Math.PI);
        double ccy = (1450 * ypos) / (9.5 * Math.PI);
        double degrees = 0;
        double error = 1;
        double erx = 1;
        double ery = 1;
        double xkr = 0.1;
        double ykr = 0.15;

        double kp = 0.085;
        while (error > 0.9 && linearOpMode.opModeIsActive()) {
            error = degrees -getAngle();
            erx = Math.abs(ccx) - Math.abs(LF.getCurrentPosition());
            ery = Math.abs(ccy) - Math.abs(RF.getCurrentPosition());
            double p = kp * error;
            double rele = kr * Math.signum(error);
            double x = Math.signum(erx) + xkr * erx;
            double y = Math.signum(ery) + ykr * ery;
            double r = p + rele;
            setMtPower(x+r, x-r, y+r, y-r);
        }
        setMtZero();
    }


    public void rotate(double degrees) {

        double ERROR = 4;
        double Er0 = -degrees;
        double errorFix=0;
        double pw = 1;
        double kr = 0.3;
        double ErLast = 0;

        while (Math.abs(ERROR)>3 && linearOpMode.opModeIsActive()) {
            ERROR  = degrees - getAngle();

            double kp = 0.6;
            double P = kp * ERROR / Er0 * pw; // P = -0.4

            double kd = 0.2;
            double ErD = ERROR - ErLast;
            //double D = kd * ErD * (1/ERROR);

            //  if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double RELE = kr * Math.signum(ERROR);
            ErLast = ERROR;

            double pwf = RELE + P;
            setMtPower(pwf, pwf, -pwf, -pwf);

            telemetry.addData("ERROR", ERROR); // хитрые проделки помидора The_IL_а
            telemetry.addData("degrees", degrees);
            telemetry.addData("getAngle", getAngle());
            telemetry.addData("RELE", RELE);
            telemetry.addData("Er0", Er0);
            telemetry.addData("pwf", pwf);
            telemetry.addData("P", P);
            telemetry.addData("pw", pw);
            telemetry.addData("lf", LF.getPower());
            telemetry.addData("lb", LB.getPower());
            telemetry.addData("rf", RF.getPower());
            telemetry.addData("rb", RB.getPower());

            telemetry.update();

        }
        setMtPower(0, 0, 0, 0);
    }

    public  void go(double cm) { //
        double pw = 1;
        double degrees = 0;
        double err = 0;
        double cc = (1450 * cm) / (9.5 * Math.PI);
        double Er0 = cc;
        double Erd = -degrees;
        double kg = 0.15;
        double errorFix=0;
        double ErLast = 0;
        double ErLast2 = 0;
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        err = -getAngle();
        double p = kg * err / Erd * pw;
        double rele = kr * Math.signum(err);
        double dr = p + rele;

        /*if (degrees < -180) {
            degrees += 360;
            pw = pw * -1;
        }
        if (degrees > 180) {
            degrees -= 360;
            pw = pw * -1;
        }*/
        double D = 1;
        double D2 = 1;
        //while (LB.getCurrentPosition() < m) { setMtPower(-pw, -pw, pw, pw); }
        while ( ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(cc)) && ( Math.abs(D) > 0.1 ||  Math.abs(cc) - Math.abs(RF.getCurrentPosition()) > 10*Math.signum(cc)) && linearOpMode.opModeIsActive()) {

            double Er = Math.abs(cc) - Math.abs(LF.getCurrentPosition());
            double Er2 = Math.abs(cc) - Math.abs(RF.getCurrentPosition());

            double kp = 0.9;
            double P = kp * Er / Er0 * pw;
            double P2 = kp * Er2 / Er0 * pw;

            double kd = 0.15;
            double ErD = Er - ErLast;
            double ErD2 = Er2 - ErLast;
            D = kd * ErD * (1 / Er);
            D2 = kd * ErD2 * (1 / Er);


            // if (Math.signum(D) > Math.signum(P)) {  D=P; }

            double kr = 0.2*Math.signum(cc);
            double Rele = kr * Math.signum(Er);
            double Rele2 = kr * Math.signum(Er2);

            ErLast = Er;
            ErLast2 = Er2;



            double pwf = (pw * (P+Rele))*Math.signum(Er);
            double pwf2 = (pw * (P2+Rele2))*Math.signum(Er2); //Регулятор

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            LF.setPower(-pwf+dr);
            RB.setPower(-pwf+dr);
            RF.setPower(-pwf2+dr);
            LB.setPower(-pwf2+dr);


            /*telemetry.addData("cc", cc);
            telemetry.addData("Er0", Er0);
            telemetry.addData("Er", Er);
            telemetry.addData("getCurrentPosition", LB.getCurrentPosition());
            telemetry.addData("kp", kp);
            telemetry.addData("Rele", Rele);
            telemetry.addData("D", D);
            telemetry.addData("pw", pw);
            telemetry.addData("pwf", pwf);
            telemetry.update();*/

        }

        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
        RF.setPower(0);

        delay(500);
    }

    public void liftOff() {
        UP1.setPower(0);
        UP2.setPower(0);
    }

    OpenCvWebcam webcam;

    public void initCamera() { // инициализация камеры
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // получение Id монитора камеры
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam"); // получение имени камеры из конфига
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId); // получение экземпляра камеры

    }


}