package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
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

    DcMotor RF, LF, LB, RB, UP1, UP2, AA; // инициализация имен моторов из конфинга RF, LF, LB, RB, AA, UP1, UP2
    Servo lift, lift2, samolet ; // инициализация имен серво из конфига up1, up2 lift, lift2 samolet, grab1, grab2
    CRServo grab1, grab2; // инциализация constant rotation servo
    BNO055IMU imu; // инициалзация иму ( внутрени сенсоров кхех)

    public static double close = 1;
    public static double lopen = 0;
    public static double lclose = 1;
   // public static double addservolift = 0.4;
    public static double dregee = 0;
    public static double kr = 0.05; // создание переменных для фтс дешборд

    Robot2023(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode linearOpMode) { // инициализация полей
        super(hardwareMap, telemetry, linearOpMode);
          LF = hardwareMap.get(DcMotor.class, "LF");
          LB = hardwareMap.get(DcMotor.class, "LB");
          RF = hardwareMap.get(DcMotor.class, "RF");
          RB = hardwareMap.get(DcMotor.class, "RB");
        UP1 = hardwareMap.get(DcMotor.class, "UP1");
        UP2 = hardwareMap.get(DcMotor.class, "UP2");
          AA = hardwareMap.get(DcMotor.class, "AA");
        grab1 = hardwareMap.get(CRServo.class, "grab1");
        grab2 = hardwareMap.get(CRServo.class, "grab2");
        lift2 = hardwareMap.get(Servo.class, "lift2");
        lift = hardwareMap.get(Servo.class, "lift");
        //up1 = hardwareMap.get(Servo.class, "up1");
        //up2 = hardwareMap.get(Servo.class, "up2");
        samolet = hardwareMap.get(Servo.class, "samolet"); // инициализация серво и моторов

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(); //Акселерометр
        parameters.angleUnit = DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
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

    public double startTick; // перменная для подсчета тиков

    double getAngle() { //Функция получения данных с акселерометра
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration gravity = imu.getGravity();
        return angles.firstAngle;
    }

    public void driveOmni() {
        double lf = (gamepad1.left_stick_y - gamepad1.left_stick_x + (gamepad1.left_trigger * 0.5) - (gamepad1.right_trigger * 0.5));
        double lb = (gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.left_trigger * 0.5) - (gamepad1.right_trigger * 0.5));
        double rf = (gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.left_trigger * 0.5) + (gamepad1.right_trigger * 0.5));
        double rb = (gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.left_trigger * 0.5) + (gamepad1.right_trigger * 0.5));
         LF.setPower(lf); // - - + + left trigger right trigger
         LB.setPower(lb);
         RF.setPower(rf);
         RB.setPower(rb); // езда с поворотами в телеопе
    }

    long map(long x, long in_min, long in_max, long out_min, long out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    } // функция мап как в ардуине

    public void drive() {
        double lf = (gamepad1.left_stick_y + gamepad1.right_trigger * 0.5 - gamepad1.left_trigger * 0.5);
        double lb = (gamepad1.left_stick_y + gamepad1.right_trigger * 0.5 - gamepad1.left_trigger * 0.5);
        double rf = (gamepad1.left_stick_y - gamepad1.right_trigger * 0.5 + gamepad1.left_trigger * 0.5);
        double rb = (gamepad1.left_stick_y - gamepad1.right_trigger * 0.5 + gamepad1.left_trigger * 0.5);
        //  LF.setPower(lf);
        //  LB.setPower(lb);
        //  RF.setPower(rf);
        //  RB.setPower(rb); // езда с поворогтами в телеопе для прямых колес


    }

    public void telemetry() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("left_y", gamepad1.left_stick_y);
        dashboardTelemetry.addData("left_x", gamepad1.left_stick_x);
        dashboardTelemetry.addData("right trigger", gamepad1.right_trigger);
        dashboardTelemetry.addData("left trigger", gamepad1.left_trigger);
        //  dashboardTelemetry.addData("salomet", samolet.getPosition());
        // dashboardTelemetry.addData("lift position", lift.getPosition());
        //  dashboardTelemetry.addData("box position", grab.getPosition());
        //  dashboardTelemetry.addData("power up1+", UP1.getPower());
        //  dashboardTelemetry.addData("power up2-", UP2.getPower());
        //  dashboardTelemetry.addData("ticks lf", LF.getCurrentPosition());
        // dashboardTelemetry.addData("power LF", LF.getPower());
        // dashboardTelemetry.addData("power LB", LB.getPower());
        // dashboardTelemetry.addData("power RF", RF.getPower());
        // dashboardTelemetry.addData("power RB", RB.getPower());
        dashboardTelemetry.addData("angle", getAngle());
        dashboardTelemetry.update(); // телеметрия для дешборда
    }

    public void init() {
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        UP1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        UP2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        AA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initCamera(); //инициализация камеры и направления моторов
    }


    public void teleOp() { // телеоп

        UP1.setPower(((gamepad2.right_stick_y) * (gamepad2.right_stick_y) * Math.signum(gamepad2.right_stick_y)));
        UP2.setPower(-((gamepad2.right_stick_y) * (gamepad2.right_stick_y) * Math.signum(gamepad2.right_stick_y))); // управление лифтом
        while (gamepad2.y) {
            double r = 0.2;
            UP1.setPower(-r);
            UP2.setPower(r);
        }  // поддержка положения лифта

        /* if (startTick < UP.getCurrentPosition()) {
        UP.setPower(-0.1);} */


       double aapower = (gamepad2.left_stick_y * gamepad2.left_stick_y) * Math.signum(gamepad2.left_stick_y); // упраление загребом
        AA.setPower(aapower);



   /* public void liftOff() {
        UP1.setPower(0);
        UP2.setPower(0);
    } */

        if (gamepad2.left_stick_y == 1) {
            grab1.setPower(close);
        } else if (gamepad2.left_stick_y == -1){
            grab1.setPower(-close);
        } else {
            grab1.setPower(0);
        }
        while (gamepad2.x) {
            grab2.setPower(-close);
        }
        while (gamepad2.a) {
            grab2.setPower(close);
        }

        if (gamepad2.right_bumper) {
            grab2.setPower(-0.05);
            grab1.setPower(0);
        } // управление коробочкой

        if (gamepad2.dpad_left) {
            samolet.setPosition(1);
        } else if (gamepad2.dpad_right) {
            samolet.setPosition(0.65);} samolet.close();// управление самолетом

        while (gamepad2.dpad_up) {
            lift.setPosition(lclose);
            lift2.setPosition(1 - lclose);
        }lift.close();
        while (gamepad2.dpad_down) {
            lift.setPosition(lopen);
            lift2.setPosition(1 - lopen);
        } lift.close();// управление механизма переворота коробочки
    /*    if (gamepad1.dpad_up) {
            up1.setPosition(0);
            up2.setPosition(1);
        } else if (gamepad1.dpad_down) {
            up1.setPosition(1);
            up2.setPosition(0);
        } */ // управление механизмом подвешивания

        telemetry.addData("right trigger: ", gamepad1.right_trigger);
        telemetry.addData("left trigger: ", gamepad1.left_trigger);
        telemetry.addData("zagreb power", AA.getPower());
        telemetry.addData("aa power", aapower);
        telemetry.addData("righttick ", gamepad2.left_stick_y);
        telemetry.addData("signum", Math.signum(gamepad2.left_stick_y));
        // telemetry.addData("encoder", startTick);
        telemetry.addData("getAngle: ", getAngle());
        //telemetry.addData("grab angles", grab.getPosition());
        telemetry.update(); // телеметрия
    }



   /* public void arm(double x, double time) {
        UP1.setPower(x);
        UP2.setPower(-x);
        delay(time);
        UP1.setPower(-0);
        UP2.setPower(0);
    } */ // поднятие лифта по времени

    public void azaxvat(double time, double direction) {
        AA.setPower(1*Math.signum(direction));
        delay(time);
        AA.setPower(0);
    } // управление загребом в автономе

   public void goTimer(double x, double y, double time) {
        LF.setPower(y - x);
        LB.setPower(y + x);
        RF.setPower(y + x);
        RB.setPower(y - x);
        delay(time);
        setMtZero();
    } // езда по времени

    public void setMtPower(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    } // мощность на каждый мотор

    public void setMtZero() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0); // отключение моторов
    }
    public void setMtAllDelay(double power, double time) {
        LF.setPower(power);
        LB.setPower(power);
        RF.setPower(power);
        RB.setPower(power);
        delay(time);
        setMtZero(); // мощность на все моторы на время
    }

    public void setMtAll(double power) {
        LF.setPower(power);
        LB.setPower(power);
        RF.setPower(power);
        RB.setPower(power); // мощеость на все моторы
    }

    public void servoClose() {
       // grab.setPosition(close);
    }

    public void servoOpen() {
       // grab.setPosition(open);
    }


    public void testpomidor() {
        double error = 2;

        double kp = 0.085;
        while (error > 0.1 && linearOpMode.opModeIsActive()) {
            error = -getAngle();
            double p = kp * error;
            double rele = kr * Math.signum(error);
            double pwf = p + rele;
             setMtPower(pwf, pwf, -pwf, -pwf); // регулятор удержания угла
        }
    }

    public void gotocoord(double xpos, double ypos) {
            LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double ccx = (1450 * xpos) / (9.5 * Math.PI);
        double ccy = (1450 * ypos) / (9.5 * Math.PI);
        double degrees = 1;
        double error = 1;
        double erx = 1;
        double ery = 1;
        double xkr = 0.08;
        double ykr = 0.08;

        double kp = 0.05;
        while (erx > 0.9 && ery > 0.9 && linearOpMode.opModeIsActive()) {
            error = degrees - getAngle();
              erx = ccx - (LF.getCurrentPosition());
              ery = ccy - (RF.getCurrentPosition());
            double p = kp * error;
            double rele = (kr + 0.1) * Math.signum(error);
            double x = Math.signum(erx) + xkr * erx;
            double y = Math.signum(ery) + ykr * ery;
            double r = p + rele;
               setMtPower( (x-r),  (y+r),  (y-r),  (x+r));
            delay(500);
        }
         setMtZero();
    } // регулятор для проезда по координатам


    public void rotate(double degrees) {

        double ERROR = 1;
        double Er0 = -degrees+0.01;
        double errorFix = 0;
        double pw = 1;
        double kr = 0.07;
        double ErLast = 0;

        while (Math.abs(ERROR) > 0.01 && linearOpMode.opModeIsActive()) {
            ERROR = degrees - getAngle();

            double kp = 0.0042;
            double P = kp * ERROR; // P = -0.4

            double kd = 0.2;
            double ErD = ERROR - ErLast;
            double D = kd * ErD * (1 / ERROR);


            double RELE = kr * Math.signum(ERROR);
            ErLast = ERROR;

            double pwf = RELE + P;
             setMtPower(pwf, pwf, -pwf, -pwf);

            telemetry.addData("ERROR", ERROR);
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

    } // регулятор для поворотов

    public void go(double cm) {
       //   setMtAllDelay(-1, 10);
        double pw = 1;
        double degrees = 0;
        double err = 1;

        double ccl = (1450 * (cm)/3) / ((9.5) * Math.PI);
        double ccr = (1450 * (cm)/3) / ((9.5) * Math.PI);
        double Erd = -degrees;
        double kg = -0.024;
        double errorFix = 0;
        double ErLast = 0;
        double ErLast2 = 0;
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        err = degrees - getAngle() + 0.00125;
        double p = kg * (err);
        double rele = (kr + 0.015) * Math.signum(err);
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
      //   while (LB.getCurrentPosition() < m) { setMtPower(-pw, -pw, pw, pw); }
        while ( (Math.abs(ccl) - Math.abs(LF.getCurrentPosition()) > 10*Math.signum(ccl)) && (Math.abs(ccr) - Math.abs(RF.getCurrentPosition()) > 10*Math.signum(ccr)) && linearOpMode.opModeIsActive()) {

            double Er = ccl - LF.getCurrentPosition();
            double Er2 =ccr - RF.getCurrentPosition();
            double cmleftls = ((Er*Math.PI * 10)/(1440));
            double cmleftrs = ((Er2*Math.PI * 10)/(1440));


            double kp = 0.00016;
            double P1 = kp * Er;
            double P2 = kp * Er2;
            double P = (P1+P2)/2;

            double kd = 0.15;
            double ErD = Er - ErLast;
            double ErD2 = Er2 - ErLast;
            D = kd * ErD * (1 / Er);
            D2 = kd * ErD2 * (1 / Er2);



            double kr = 0.04;
            double Rele1 = kr * Math.signum(Er);
            double Rele2 = kr * Math.signum(Er2);
            double Rele = (Rele1 + Rele2)/2;

            ErLast = Er;
            ErLast2 = Er2;



           // double pc = 1-dr;
            double pwf1 = (pw * (P1+Rele1));
            double pwf = (pw * (P+Rele));
            double pwf2 = (pw * (P2+Rele2));//Регулятор

            //telemetry.addData("currPosition", LF.getCurrentPosition());
            //telemetry.addData("cc", cc);
            //telemetry.addData("Err", Er);
            //telemetry.addData("cond1", cc - LF.getCurrentPosition());
            //telemetry.addData("cond2", 10*Math.signum(cc));
            //telemetry.addData("pwf", pwf);
            //telemetry.addData("D", D);
            //telemetry.update();

            LF.setPower((pwf1));
            LB.setPower((pwf2));
            RB.setPower((pwf1));
            RF.setPower((pwf2));


           /*  while (cmleftrs > cmleftls) {
                RF.setPower(0.05);
                RB.setPower(0.05);
            } while (cmleftls > cmleftrs) {
                LF.setPower(0.05);
                LB.setPower(0.05);
            } */


            telemetry.addData("ccl", ccl);
            telemetry.addData("ccr", ccr);
            telemetry.addData("lf",LF.getPower());
            telemetry.addData("lb",LB.getPower());
            telemetry.addData("rb",RB.getPower());
            telemetry.addData("rf",RF.getPower());
            telemetry.addData("pwf", pwf);
            telemetry.addData("cm left left side", cmleftls);
            telemetry.addData("cm left right side", cmleftrs);
            telemetry.addData("angle", getAngle());
            telemetry.addData("p", P);
            telemetry.addData("rere", Rele);
            telemetry.addData("Er", Er);
            telemetry.addData("pw", pw);
            telemetry.addData("dr", dr);
            telemetry.addData("pwf2", pwf);
            telemetry.addData("err", err);
            telemetry.addData("poweerlf", LF.getPower());
            telemetry.update();

        }

        setMtZero();


        delay(500);
    } // езда по экнодерам с удержанием угла

    public void ngo(double cc) {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double err = 1;
        while ( (Math.abs(cc) - Math.abs(LF.getCurrentPosition()) > 2*Math.signum(cc))) {
            double rele = kr * Math.signum(err);
            double error = cc - LF.getCurrentPosition();
            double kr = 0.05;
            double kp = 0.0004;
            boolean ief = false;
            double p = kp*error;
            double pwf = rele + p;
            double sa = getAngle();
            while (Math.abs(err)>0.1) {
                ief = false;
                err = sa - getAngle();
                double ep = 0.03;
                double er = 0.05;
                double ap = ep*err;
                double ar = Math.signum(err)*er;
                double pa = ap + ar;
                LF.setPower(-pa);
                LB.setPower(pa);
                RF.setPower(pa);
                RB.setPower(-pa);
                ief = true;
                if (Math.abs(err) > 1) ief = false;
            }
            if (ief) {
                LF.setPower(-pwf);
                LB.setPower(-pwf);
                RF.setPower(-pwf);
                RB.setPower(-pwf);
                delay(200);
            }
            telemetry.addData("cc", cc);
            telemetry.addData("lf",LF.getPower());
            telemetry.addData("lb",LB.getPower());
            telemetry.addData("rb",RB.getPower());
            telemetry.addData("rf",RF.getPower());
             telemetry.addData("angle", getAngle());
            telemetry.addData("poweerlf", LF.getPower());
            telemetry.update();

        }
        setMtZero();
    }


        OpenCvWebcam webcam;

        public void initCamera() { // инициализация камеры
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            // получение Id монитора камеры
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam"); // получение имени камеры из конфига
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId); // получение экземпляра камеры

        }


    }