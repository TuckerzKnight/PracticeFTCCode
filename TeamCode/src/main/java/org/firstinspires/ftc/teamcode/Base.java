/*
Base handles bulk reading, and hub voltage
 */

package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Config
public class Base {
    //configurable presets

    public static double pixelHeight = 12.7;
    public static double clawTravel = 100; //in servo degrees. The current setup causes 720/7 degrees of motion per revolution
    public static double wristTravel = 230;
    public static int grabTime = 500;
    public static double rollTravel = (270*(14.0/20));
    public static double stallDutyCycle = 0.3; //If the program is running conditions prone to stalling, restrict "on" percentage to:
    public static double safeStall = 3; //The amount of time it will run until cooldown, in seconds. This should be long enough to home
    public static int depositTime = 1000; //in millis
    private FtcDashboard dashboard;
    public static boolean alliance = false; //This determines how to zero gyro. false = red, true = blue

    public static double rollRatio = (9/20.0); //9t pinion
    public static double servoSpeed = (0.1928*1.33333); //degrees per millisecond, REV's documentation converted to actual findings

    //The length of each segment of the arm. Adding an additional value adds an additional segment, but please note only revolute joints
    //on a 2D plane are used, and inverse kinematics are not scalable like forward kinematics
    //see array jointPositions in loop to input your joint motors
    public static double [] links = {319, 269.19, 112};

    VoltageSensor myControlHubVoltageSensor;
    public double presentVoltage;

    public IMU imu;

    //button debounces, so each press only registers once
    boolean resetDown = false;
    boolean dDpadDebounce = false;
    boolean overrideDebounce = false;

    public static double powerMultiplier;

    public static int voltageHistoryLength = 6;
    public static double[] voltageHistory = new double[voltageHistoryLength]; //store the last few values of control hub voltage
    double averageVoltage;
    int cycleNum = 0;
    Servo dr, dl, roll;
    DcMotorEx lf, lr, rf, rr, sl, sr, el, er;

    DigitalChannel rollSwitch;
    public static double lfPos, lrPos, rfPos, rrPos, slPos, srPos, elPos, erPos, rollPos = 0, drPos, dlPos;
    public static double [] jointPositions;
    public static double lfPwr, lrPwr, rfPwr, rrPwr, shoulderPwr, elbowPwr, rollPwr;
    public static DcMotor.ZeroPowerBehavior zeroPower = DcMotor.ZeroPowerBehavior.BRAKE;

    public static boolean clawState = false;

    public static double wristPitch;

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
    public static double shoulderTarget, elbowTarget, rollTarget, wristTarget;

    double lastShoulderTarget, lastElbowTarget;

    //store how long the servo has been moving to approximate position
    ElapsedTime rollTimer = new ElapsedTime();

    int rollFailureCounter = 0;

    public static int wristTolerance =  1;
    public static boolean rollState = false;

    //State configuration

    //These do not actually require initialization, due to zero degrees equaling zero ticks
    //Use FTCDashboard to tune these guys, because they don't need initialization
    public static double [] idlePos = {-5, 0, 210, 0.02};
    public static double [] intakePos = {-5, 0, -70, 0.02};

    //leave these with just the desired position and run armIK, or comment out the function and predefine angle in degrees
    public static double [] backtakePos = {170, -125, 180};
    public static double [] stackPos = {145, -125, 0};//for intaking only the y and angle of the claw are required
    public static double [] lowerStackPos = {160, -125, 0};
    public static double depositHeight = 450; //remember height across deposits

    //array structure: shoulder angle, elbow angle, wrist pitch, wrist roll
    public static double [] depositPos = {67, 137, 40, 0}; //figure out actual board angle sometime
    public static double [] backDepositPos = {30, 170, 70, 180};

    //in any application calculating kinematics on the fly, it is useful to operate on the base inputs
    double[] initialDepositPos = {depositPos[0],depositPos[1],depositPos[2],depositPos[3]};
    double[] initialBackDepositPos = {backDepositPos[0],backDepositPos[1],backDepositPos[2],backDepositPos[3]};

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
    public static State state;

    ElapsedTime stateTimer = new ElapsedTime();

    boolean xDebounce = false;
    boolean aDebounce = false;
    boolean dpadDebounce = false;
    double elbowError, shoulderError;
    boolean Default = false;
    boolean stackHeight = false;


    public static int errorCode = 0;

    //hardware mapping and the like
    public void init (HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        dashboard = FtcDashboard.getInstance();
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        //global hardware definitions
        /*roll = hardwareMap.get(Servo.class, "rollServo");
        dl = hardwareMap.get(Servo.class, "diffyLeft");
        dr = hardwareMap.get(Servo.class, "diffyRight");*/
        lf = hardwareMap.get(DcMotorEx.class, "frontLeft");
        rf = hardwareMap.get(DcMotorEx.class, "frontRight");
        lr = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rr = hardwareMap.get(DcMotorEx.class, "rearRight");
        /*sl = hardwareMap.get(DcMotorEx.class, "shoulderLeft");
        sr = hardwareMap.get(DcMotorEx.class, "shoulderRight");
        el = hardwareMap.get(DcMotorEx.class, "elbowLeft");
        er = hardwareMap.get(DcMotorEx.class, "elbowRight");*/


        //rollSwitch = hardwareMap.get(DigitalChannel.class, "rollSwitch");

        imu = hardwareMap.get(IMU.class, "imu");

        //rollSwitch.setMode(DigitalChannel.Mode.INPUT);

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
        /*er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*sl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/

        lr.setZeroPowerBehavior(zeroPower);
        rf.setZeroPowerBehavior(zeroPower);
        lf.setZeroPowerBehavior(zeroPower);
        rr.setZeroPowerBehavior(zeroPower);

        state = State.IDLE;

        //zero out claw.
        /*dl.setPosition(0);
        dr.setPosition(0);
        roll.setPosition(0);*/

    }

    //get hardware input, write hardware output
    public void update (Telemetry telemetry) {
        lfPos = lf.getCurrentPosition();
        lrPos = lr.getCurrentPosition();
        rfPos = rf.getCurrentPosition();
        rrPos = rr.getCurrentPosition();
        /*slPos = -sl.getCurrentPosition();
        srPos = sr.getCurrentPosition();
        elPos = el.getCurrentPosition();
        erPos = er.getCurrentPosition();*/

        //rollState = !rollSwitch.getState(); //rollSwitch.getState = false when triggered, true when not

        //kinematics formatting, input your actuator position (or average multiple)
        //jointPositions = new double [] {((slPos + srPos)/2), ((elPos + erPos)/2), wristPitch}; //lol beware, if garbage collector doesn't catch this then memory leak

        lf.setPower(lfPwr);
        lr.setPower(lrPwr);
        rf.setPower(rfPwr);
        rr.setPower(rrPwr);
        /*sl.setPower(-shoulderPwr);
        sr.setPower(shoulderPwr);
        el.setPower(elbowPwr);
        er.setPower(elbowPwr);

        roll.setPosition(rollPos);
        dl.setPosition(dlPos);
        dr.setPosition(drPos);*/

        presentVoltage = myControlHubVoltageSensor.getVoltage();

        //get the average voltage of the past 5 cycles
        averageVoltage = 0;
        for (int i = 0; i < voltageHistoryLength; i++) {
            averageVoltage += voltageHistory[i];
        }
        averageVoltage /= voltageHistoryLength;
        powerMultiplier = min(presentVoltage / averageVoltage, 1);
        voltageHistory[cycleNum%voltageHistoryLength] = presentVoltage;
        cycleNum++;

        /*if (errorCode == 1) {
            telemetry.addLine("Default!");
        } else if (errorCode == 2) {
            telemetry.addLine("Roll range IK failed");
        }
        telemetry.addData("elTicks", elPos);
        telemetry.addData("erTicks", erPos);
        telemetry.addData("elbowError",elbowError);
        telemetry.addData("elbowTarget", elbowTarget);
        telemetry.addData("elbowPwr", elbowPwr);
        telemetry.addData("shoulderError", shoulderError);
        telemetry.addData("shoulderTarget", shoulderTarget);
        telemetry.addData("shoulderPwr", shoulderPwr);*/
        telemetry.addData("state", state);
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        /*packet.put("elbowTarget", elbowTarget);
        packet.put("el ticks:", elPos);*/
        packet.put("heading", DriveHardware.heading);
        /*packet.put("clawState", clawState);
        packet.fieldOverlay()
                        .setFill("blue")
                                .fillRect((Kinematics.X-Kinematics.robotwidth),(Kinematics.Y-Kinematics.robotwidth),40,40)
                                        .strokeCircle(100,100,5);*/
        dashboard.sendTelemetryPacket(packet);

    }

    //Move actuators to targets
    public void motorManage () {
        //if manual override mode, ignore system code
        if (state == State.OVERRIDE){
            return;
        }
        /*
        //if the limit switch triggered, it is at zero, but if it thinks its zeroed and its not it will continue to oscillate
        if (rollState){
            rollPos = 0;
            rollFailureCounter = 0;
        } else if (rollPos == 0) { //if the approximation says it should be at zero but hardware says it isn't, overshoot a bit to compensate
            //each oscillation is bigger until maximum exits loop
            if (rollTimer.milliseconds() > (50*(rollFailureCounter+1))) {
                rollTarget += rollPwr*rollTimer.milliseconds()*servoSpeed*rollRatio;
                //if oscillating could not zero, attempt full rehoming
                if (rollFailureCounter > 6) {
                    rollTarget = 180;
                }
                rollFailureCounter = (rollFailureCounter*rollFailureCounter)+1;
            }
        }
        //unless failing to home, go to position
            //if the target and current positions are close enough, exit
            if (rollTarget < (rollPos - wristTolerance)) {
                //if the target is below the current value with a little tolerance for error, then:
                if (rollPwr == 0) {
                    rollPwr = -1;
                    rollTimer.reset();
                } else {
                    //Add approximate change in servo position
                    rollPos += rollPwr * rollTimer.milliseconds() * servoSpeed * rollRatio;
                    rollTimer.reset();
                }
            } else if (rollTarget > (rollPos + wristTolerance)) {
                //if the servo hasn't already been started, start it
                if (rollPwr == 0) {
                    rollPwr = 1;
                    rollTimer.reset();
                } else {
                    //Add approximate change in servo position
                    rollPos += rollPwr * rollTimer.milliseconds() * servoSpeed * rollRatio;
                    rollTimer.reset();
                }
            } else {
                rollPwr = 0;
        }*/

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

    //put action code here, like if you want a specific movement in depositing or smth
    public void stateMachine (Gamepad gamepad1,Gamepad gamepad2, Telemetry telemetry) {
        //put code here that should be run regardless of state:

        //if dpad up on either gamepad is pressed, toggle manual override
        if (!overrideDebounce && (gamepad2.dpad_up || gamepad1.dpad_up)) {
            state = State.OVERRIDE;
            overrideDebounce = true;
        } else if (overrideDebounce && !(gamepad2.dpad_up || gamepad1.dpad_up)) { // if neither dpad, un-debounce
            overrideDebounce = false;
        }
        if (!dpadDebounce) {
            if (gamepad1.dpad_down) {
                alliance = !alliance;
                dpadDebounce = true;
            } else if (gamepad1.dpad_left) {
                imu.resetYaw(); //zero out gyro
                DriveHardware.timesAcrossZero = 0; //zero out software
                DriveHardware.desiredHeading = 0;
                Kinematics.X = 0;
                Kinematics.Y = 0;
                Kinematics.heading = 0;
                dpadDebounce = true;
            } else if (gamepad1.dpad_right) {
                DriveHardware.fieldCentric = !DriveHardware.fieldCentric;
                dpadDebounce = true;
            }
        } else if (!(gamepad1.dpad_left || gamepad1.dpad_right || gamepad1.dpad_down)) {
            dpadDebounce = false;
        }
        if (!gamepad1.a) {
            aDebounce = false;
        }
        //state-specific code, don't forget break;!
        switch (state){
            case IDLE:
                shoulderTarget = idlePos[0];
                elbowTarget = idlePos[1];
                wristTarget = idlePos[2];
                rollPos = idlePos[3];
                if (gamepad1.right_bumper) {
                    state = State.INTAKE;
                    clawState = false;
                } else if (gamepad1.left_bumper) {
                    state = State.BACKTAKE;
                    clawState = false;
                } else if (gamepad1.x) {
                    state = State.BACKPUT;
                    clawState = true;
                } else if (gamepad1.y) {
                    stateTimer.reset();
                    state = State.DEPOSIT;
                    clawState = true;
                } else if (gamepad1.a && !aDebounce) {
                    clawState = !clawState;
                    aDebounce = true;
                }
                break;
            case INTAKE:
                shoulderTarget = intakePos[0];
                elbowTarget = intakePos[1];
                wristTarget = intakePos[2];
                rollPos = intakePos[3];

                if (gamepad1.a && clawState == false && aDebounce == false) {
                    clawState = true;
                    aDebounce = true;
                    stateTimer.reset();
                } else if (clawState == true && stateTimer.milliseconds() > grabTime) {
                    state = State.IDLE;
                } else if (gamepad1.b) {
                    clawState = false;
                    state = State.IDLE;
                }
                break;
            case STAKK: //intake from auto stacks, make sure to use normal intake if there's just one left
                if (stackHeight) {
                    shoulderTarget = 0;
                    elbowTarget = stackPos[0];
                    wristTarget = stackPos[1];
                    rollPos = stackPos[2];
                } else {
                    shoulderTarget = 0;
                    elbowTarget = lowerStackPos[0];
                    wristTarget = lowerStackPos[1];
                    rollPos = lowerStackPos[2];
                }

                if (gamepad1.a && clawState == false && aDebounce == false) {
                    clawState = true;
                    aDebounce = true;
                    stateTimer.reset();
                } else if (clawState == true && stateTimer.milliseconds() > grabTime) {
                    state = State.IDLE;
                    stackHeight = !stackHeight;
                } else if (gamepad1.b) {
                    clawState = false;
                    state = State.IDLE;
                } else if (gamepad1.x && !xDebounce) {//select stack height
                    xDebounce = true;
                    stackHeight = !stackHeight;
                }
                if (!gamepad1.x) {
                    xDebounce = false;
                }
                break;
            case BACKTAKE:
                shoulderTarget = 0;
                elbowTarget = backtakePos[0];
                wristTarget = backtakePos[1];
                rollPos = backtakePos[2];
                if (gamepad1.a && clawState == false && aDebounce == false) {
                    clawState = true;
                    aDebounce = true;
                    stateTimer.reset();
                } else if (clawState == true && stateTimer.milliseconds() > grabTime) {
                    state = State.IDLE;
                } else if (gamepad1.b) {
                    clawState = false;
                    state = State.IDLE;
                }
                break;
            case DEPOSIT:
                shoulderTarget = depositPos[0];
                elbowTarget = depositPos[1];
                wristTarget = depositPos[2];
                rollPos = depositPos[3];
                //if you haven't hit deposit yet, look for pressing controls
                if (clawState) {
                    //use right stick y to (hopefully) increase deposit height
                    depositPos[0] += gamepad1.right_stick_y;
                    depositPos[1] += gamepad1.right_stick_y;
                    depositPos[2] += gamepad1.right_stick_y;
                    if (gamepad1.a && !aDebounce) {
                        clawState = false;
                        aDebounce = true;
                        wristTarget -= 30; //dump pixel
                        stateTimer.reset();
                    } else if (gamepad1.b) {
                        state = State.IDLE;
                    }
                } else if (stateTimer.milliseconds() > depositTime) {
                    state = State.IDLE;
                }
                /*
                //pick which deposit is desired based on IMU heading
                //if red alliance and facing right of driver, deposit forward
                    if (DriveHardware.heading >= 180) {
                        shoulderTarget = depositPos[0];
                        elbowTarget = depositPos[1];
                        wristTarget = depositPos[2];
                        rollPos = depositPos[3];

                        //if a, deposit and zero IMU
                        if (clawState == true) {
                            if (gamepad1.a && aDebounce == false) {
                                clawState = false;
                                aDebounce = true;
                                wristTarget -= 30; //dump pixel
                                stateTimer.reset();
                                imu.resetYaw(); //zero out gyro
                                DriveHardware.timesAcrossZero = 0; //zero out software
                                DriveHardware.desiredHeading = 0;
                                Kinematics.heading = 0;
                            } else if (gamepad1.b) {
                                state = State.IDLE;
                            } else if (gamepad1.right_stick_y != 0 && !((gamepad1.right_stick_y < 0 && depositHeight < 50) || (gamepad1.right_stick_y > 0 && depositHeight > 550))){//reach limits
                                //Using this requires recalculating kinematics!
                                depositHeight += gamepad1.right_stick_y*5; //scalable constant for adjustment speed, at mm/refresh
                                if (!alliance) {
                                    //depositPos[0] = initialDepositPos[0]*(sin(depositPos[2]) * depositHeight); //shift claw position along claw plane
                                    //depositPos[1] = initialDepositPos[1]*(cos(depositPos[2]) * depositHeight);
                                    //run kinematics as an if statement so if it returns false take action
                                    //if (!Kinematics.armIK(depositPos[0], depositPos[1], depositPos[2], telemetry)) {
                                        //Okay if kinematics failed quick undo everything I just did
                                        //depositHeight -= gamepad1.right_stick_y * 5; //scalable constant for adjustment speed, at mm/refresh
                                        //depositPos[0] = initialDepositPos[0]*(sin(depositPos[2]) * depositHeight);
                                        //depositPos[1] = initialDepositPos[1]*(cos(depositPos[2]) * depositHeight);
                                    //}

                                    //While kinematics are down, this should suffice
                                    depositPos[0] += 5*Kinematics.shoulderRatio;
                                    depositPos[1] += 5*Kinematics.elbowRatio;//if you multiply them by the ratio, then they should both move the same distance, therefore going up
                                } else {
                                    //While kinematics are down, this should suffice
                                    backDepositPos[0] += 5*Kinematics.shoulderRatio;
                                    backDepositPos[1] += 5*Kinematics.elbowRatio;//if you multiply them by the ratio, then they should both move the same distance, therefore going up
                                }
                            }
                        } else if (stateTimer.milliseconds() > 750){
                            state = State.IDLE;
                        }
                    } else {
                        shoulderTarget = backDepositPos[0];
                        elbowTarget = backDepositPos[1];
                        wristTarget = backDepositPos[2];
                        rollPos = backDepositPos[3];

                        if (clawState == true) {
                            if (gamepad1.a) {
                                clawState = false;
                                wristTarget -= 30; //dump pixel
                                stateTimer.reset();
                                //if depositing, with the current code that means you are parallel to the board, so we know yaw
                                imu.resetYaw(); //zero out gyro
                                DriveHardware.timesAcrossZero = 0; //zero out software
                                DriveHardware.desiredHeading = 0;
                                Kinematics.heading = 0;
                            } else if (gamepad1.b) {
                                state = State.IDLE;
                            } else if (gamepad1.right_stick_y != 0 && !((gamepad1.right_stick_y < 0 && depositHeight < 50) || (gamepad1.right_stick_y > 0 && depositHeight > 550))){//reach limits
                                //Using this requires recalculating kinematics!
                                depositHeight += gamepad1.right_stick_y*5; //scalable constant for adjustment speed, at mm/refresh
                                if (!alliance) {
                                    //While kinematics are down, this should suffice
                                    backDepositPos[0] += 5*Kinematics.shoulderRatio;
                                    backDepositPos[1] += 5*Kinematics.elbowRatio;//if you multiply them by the ratio, then they should both move the same distance, therefore going up
                                } else {
                                    //While kinematics are down, this should suffice
                                    depositPos[0] += 5*Kinematics.shoulderRatio;
                                    depositPos[1] += 5*Kinematics.elbowRatio;//if you multiply them by the ratio, then they should both move the same distance, therefore going up
                                }
                            }
                        } else if (stateTimer.milliseconds() > 750){
                            state = State.IDLE;
                        }

                }*/
                break;
            case BACKPUT:
                shoulderTarget = backDepositPos[0];
                elbowTarget = backDepositPos[1];
                wristTarget = backDepositPos[2];
                rollPos = backDepositPos[3];
                //if you haven't hit deposit yet, look for pressing controls
                if (clawState) {
                    //use right stick y to (hopefully) increase deposit height
                    backDepositPos[0] += gamepad1.right_stick_y;
                    backDepositPos[1] += gamepad1.right_stick_y;
                    backDepositPos[2] += gamepad1.right_stick_y;
                    if (gamepad1.a && !aDebounce) {
                        clawState = false;
                        aDebounce = true;
                        wristTarget -= 30; //dump pixel
                        stateTimer.reset();
                    } else if (gamepad1.b) {
                        state = State.IDLE;
                    }
                } else if (stateTimer.milliseconds() > depositTime) {
                    state = State.IDLE;
                }
            case OVERRIDE:
                //manual control over all functions
                shoulderPwr = gamepad2.left_stick_y;
                elbowPwr = gamepad2.left_stick_x;
                rollPwr = gamepad2.right_stick_x;
                int clawPower = 0;
                if (gamepad2.a) {
                    clawPower = 1;
                } else if (gamepad2.b){
                    clawPower = 0;
                }
                drPos = Range.clip(-gamepad2.right_stick_y-clawPower, 0, 1);
                dlPos = Range.clip(gamepad2.right_stick_y-clawPower, 0, 1);

                //if dpad pressed, reset arm encoders. Perhaps also zero out servos, but most of those are on endstops anyway
                if (!resetDown && gamepad2.dpad_left) {
                    sl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    el.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    er.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    er.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    el.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    sl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    sr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    resetDown = true;
                } else if (!gamepad2.dpad_left) {
                    resetDown = false;
                }
                break;
            default:
                //if sent to an invalid state, return to IDLE
                state = State.IDLE;
                errorCode = 1;
        }
    }
}
