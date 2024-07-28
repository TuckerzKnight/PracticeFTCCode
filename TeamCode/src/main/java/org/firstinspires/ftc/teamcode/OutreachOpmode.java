package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

@TeleOp(name = "OutreachOpmode", group = "WIP")
public class OutreachOpmode extends LinearOpMode {


    public static double clawTravel = 100; //in servo degrees. The current setup causes 720/7 degrees of motion per revolution
    public static int grabTime = 500;
    public static int depositTime = 1000; //in millis
    private FtcDashboard dashboard;

    public IMU imu;
    Servo dr, dl, roll;
    DcMotorEx lf, lr, rf, rr, sl, sr, el, er;

    public static double lfPos, lrPos, rfPos, rrPos, slPos, srPos, elPos, erPos, rollPos = 0, drPos, dlPos;
    public static double lfPwr, lrPwr, rfPwr, rrPwr, shoulderPwr, elbowPwr;
    public static DcMotor.ZeroPowerBehavior zeroPower = DcMotor.ZeroPowerBehavior.BRAKE;

    public static boolean clawState = false;


    public static double sKp = 0.015;//PID constants for shoulder and elbow positioning
    public static double sKi = 0.1;
    public static double sKd = 0;
    public static double eKp = 0.01;
    public static double eKi = 0.01;
    public static double eKd = 0.001;
    double shoulderIntegralSum = 0, elbowIntegralSum = 0;
    double shoulderLastError = 0, elbowLastError = 0;
    ElapsedTime shoulderPID = new ElapsedTime();
    ElapsedTime elbowPID = new ElapsedTime();
    public static double shoulderTarget, elbowTarget, wristTarget;

    double lastShoulderTarget, lastElbowTarget;

    //State configuration

    //These do not actually require initialization, due to zero degrees equaling zero ticks
    //Use FTCDashboard to tune these guys, because they don't need initialization
    public static double [] idlePos = {-5, 0, 210, 0.02};
    public static double [] intakePos = {-5, 0, -70, 0.02};
    public static double [] backDepositPos = {30, 170, 70, 180};
    public static double cooldown = 2; // in seconds, how long to wait before allowing another state machine switch

    ElapsedTime cooldownTimer = new ElapsedTime();

    //define states for state machine
    public enum State {
        IDLE,
        DEPOSIT,
        BACKTAKE,
        OVERRIDE,
        STAKK,
        INTAKE,
        BACKPUT
    }
    public static Base.State state;

    ElapsedTime stateTimer = new ElapsedTime();
    boolean aDebounce = false;
    double elbowError, shoulderError;


    boolean yDebounce = false;
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        dashboard = FtcDashboard.getInstance();

        //global hardware definitions
        roll = hardwareMap.get(Servo.class, "rollServo");
        dl = hardwareMap.get(Servo.class, "diffyLeft");
        dr = hardwareMap.get(Servo.class, "diffyRight");
        lf = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rf = hardwareMap.get(DcMotorEx.class, "frontRight");
        lr = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rr = hardwareMap.get(DcMotorEx.class, "rearRight");
        sl = hardwareMap.get(DcMotorEx.class, "shoulderLeft");
        sr = hardwareMap.get(DcMotorEx.class, "shoulderRight");
        el = hardwareMap.get(DcMotorEx.class, "elbowLeft");
        er = hardwareMap.get(DcMotorEx.class, "elbowRight");



        imu = hardwareMap.get(IMU.class, "imu");

        //Here you can edit the individual motor headings
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);
        //theres a good chance none of this is necessary
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lr.setZeroPowerBehavior(zeroPower);
        rf.setZeroPowerBehavior(zeroPower);
        lf.setZeroPowerBehavior(zeroPower);
        rr.setZeroPowerBehavior(zeroPower);

        state = Base.State.IDLE;

        //zero out claw.
        dl.setPosition(0);
        dr.setPosition(0);
        roll.setPosition(0);

        telemetry.addLine("Robot initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            lfPos = lf.getCurrentPosition();
            lrPos = lr.getCurrentPosition();
            rfPos = rf.getCurrentPosition();
            rrPos = rr.getCurrentPosition();
            slPos = -sl.getCurrentPosition();
            srPos = sr.getCurrentPosition();
            elPos = el.getCurrentPosition();
            erPos = er.getCurrentPosition();

            lf.setPower(lfPwr);
            lr.setPower(lrPwr);
            rf.setPower(rfPwr);
            rr.setPower(rrPwr);
            sl.setPower(-shoulderPwr);
            sr.setPower(shoulderPwr);
            el.setPower(elbowPwr);
            er.setPower(elbowPwr);

            roll.setPosition(rollPos);
            dl.setPosition(dlPos);
            dr.setPosition(drPos);

            //all this turn stuff is to square turning, to allow easier control
            boolean turnSign = gamepad1.right_stick_x >= 0;
            double turn = (gamepad1.right_stick_x*gamepad1.right_stick_x)*-0.50;
            if (!turnSign) {
                turn = turn*-1;
            }
            double straight = gamepad1.left_stick_y*0.5;
            double strafe = gamepad1.left_stick_x*0.5;
            lfPwr = (turn + straight - strafe);
            rfPwr = (straight - turn + strafe);
            lrPwr = (straight + turn + strafe);
            rrPwr = (straight - turn - strafe);


            telemetry.addData("elTicks", elPos);
            telemetry.addData("erTicks", erPos);
            telemetry.addData("elbowError",elbowError);
            telemetry.addData("elbowTarget", elbowTarget);
            telemetry.addData("elbowPwr", elbowPwr);
            telemetry.addData("shoulderError", shoulderError);
            telemetry.addData("shoulderTarget", shoulderTarget);
            telemetry.addData("shoulderPwr", shoulderPwr);
            telemetry.addData("state", state);
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("elbowTarget", elbowTarget);
            packet.put("el ticks:", elPos);
            packet.put("heading", DriveHardware.heading);
            packet.put("clawState", clawState);
            packet.fieldOverlay()
                    .setFill("blue")
                    .fillRect((Kinematics.X-Kinematics.robotwidth),(Kinematics.Y-Kinematics.robotwidth),40,40)
                    .strokeCircle(100,100,5);
            dashboard.sendTelemetryPacket(packet);

            if (!gamepad1.a) {
                aDebounce = false;
            }
            if (!gamepad1.y) {
                yDebounce = false;
            }
            //state-specific code, don't forget break;!
            switch (state) {
                case IDLE:
                    shoulderTarget = idlePos[0];
                    elbowTarget = idlePos[1];
                    wristTarget = idlePos[2];
                    rollPos = idlePos[3];
                    if (cooldownTimer.seconds() > cooldown) {
                        if (gamepad1.a && aDebounce == false) {
                            state = Base.State.INTAKE;
                            cooldownTimer.reset();
                            clawState = false;
                            aDebounce = true;
                        } else if (gamepad1.y && yDebounce == false) {
                            state = Base.State.BACKPUT;
                            cooldownTimer.reset();
                            yDebounce = true;
                            clawState = true;
                        }
                    }
                    break;
                case INTAKE:
                    shoulderTarget = intakePos[0];
                    elbowTarget = intakePos[1];
                    wristTarget = intakePos[2];
                    rollPos = intakePos[3];
                    if (cooldownTimer.seconds() > cooldown) {
                        if (gamepad1.a && clawState == false && aDebounce == false) {
                            clawState = true;
                            aDebounce = true;
                            stateTimer.reset();
                        } else if (clawState == true && stateTimer.milliseconds() > grabTime) {
                            state = Base.State.IDLE;
                            cooldownTimer.reset();
                        }
                    }
                    break;
                case BACKPUT:
                    shoulderTarget = backDepositPos[0];
                    elbowTarget = backDepositPos[1];
                    wristTarget = backDepositPos[2];
                    rollPos = backDepositPos[3];
                    if (cooldownTimer.seconds() > cooldown) {
                        //if you haven't hit deposit yet, look for pressing controls
                        if (clawState) {
                            //use right stick y to (hopefully) increase deposit height
                            backDepositPos[0] += gamepad1.right_stick_y;
                            backDepositPos[1] += gamepad1.right_stick_y;
                            backDepositPos[2] += gamepad1.right_stick_y;
                            if (gamepad1.y && !yDebounce) {
                                clawState = false;
                                yDebounce = true;
                                wristTarget -= 30; //dump pixel
                                stateTimer.reset();
                            }
                        } else if (stateTimer.milliseconds() > depositTime) {
                            state = Base.State.IDLE;
                            cooldownTimer.reset();
                        }
                    }
                    break;
                default:
                    //if sent to an invalid state, return to IDLE
                    state = Base.State.IDLE;
            }
            drPos = Range.clip(((wristTarget/360.0)/2)+0.5,0,1);
            dlPos = Range.clip((-(wristTarget/360.0)/2)+0.5,0,1);//convert from degrees to 0-1
            if (!clawState) {
                drPos -= ((clawTravel / 270.0) / 2);
                dlPos -= ((clawTravel / 270.0) / 2);
            } else {
                drPos += 0.05;
                dlPos += 0.05;//if it doesn't open all the way, heres an easy place to fix that
            }
            //Okie motor PID stuff here
            //driveHardware computes drive motors, so here we just bring arm motors to position

            //CTRL + ALT + FTC PID
            shoulderError = shoulderTarget - (((slPos + srPos)/2)*360/(Kinematics.shoulderRatio*28));
            shoulderIntegralSum = Range.clip(shoulderIntegralSum + (shoulderError * shoulderPID.seconds()),-1,1);
            shoulderPwr = (sKp * shoulderError) + (sKi * shoulderIntegralSum) + (sKd * ((shoulderError-shoulderLastError)/shoulderPID.seconds()));
            shoulderLastError = shoulderError;
            shoulderPID.reset();
            if (lastShoulderTarget != shoulderTarget) {
                shoulderIntegralSum = 0;
            }
            lastShoulderTarget = shoulderTarget;

            elbowError = elbowTarget - ((((elPos + erPos)/2)+(slPos +srPos)/(2*1.4))*360/(Kinematics.elbowRatio*28));
            elbowIntegralSum = Range.clip(elbowIntegralSum + (elbowError * elbowPID.seconds()),-1,1);
            elbowPwr = (eKp * elbowError) + (eKi * elbowIntegralSum) + (eKd * ((elbowError-elbowLastError)/elbowPID.seconds()));
            elbowLastError = elbowError;
            elbowPID.reset();
            if (lastElbowTarget != elbowTarget) {
                elbowIntegralSum = 0;
            }
            lastElbowTarget = elbowTarget;
        }
    }
}
