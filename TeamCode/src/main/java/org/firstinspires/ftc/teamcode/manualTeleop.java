
package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



@Config
@TeleOp(name = "GG{GetthouGooder}")
public class manualTeleop extends LinearOpMode {

    //these guys are for ftcdashboard
    public static double dlOVERRIDE = 0;
    public static double drOVERRIDE = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        //define motors as existing
        DcMotorEx lr, rf, lf, rr, sl, sr, el, er;

        IMU imu;

         double heading;

         double desiredHeading;

        int timesAcrossZero;

        double fixedHeading;

        Orientation angles;
        boolean dpadDebounce = false;

        boolean justTurned = false;

        double transientSpeed;

        double  transientTurn = 0;

        double rampSpeed = 0.2; //Rate in percentage at which function ramps to speed

         double turnOverride = 0.75; //at one turning can completely override drive, at zero turning has no effect

         double turnRamp = 0.5; //this can be adjusted to slow down or speed up the turning acceleration

         double headingCorrectionRange = 45; //this scales the reaction of the heading correction-
        // smaller values encourage snappy reaction, but larger values decrease overshoot

        double leftDiagPwr;
        double rightDiagPwr;
        Servo dl, dr, roll;

         boolean fieldCentric = true;
        boolean justTurbo = false;
        //cool
        imu = hardwareMap.get(IMU.class, "imu");
        angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //poll IMU
        heading = Range.scale(angles.firstAngle, 0, 360, -1, 1); //extract heading
        imu.resetYaw();

        timesAcrossZero = 0;
        transientSpeed = 0;
        desiredHeading = 0;

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

        //Here you can edit the individual motor headings
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        double rollPwr = 0;

        boolean clawState = false;

        boolean rbDebounce = false;

        double drPos = 0;
        double dlPos = 0;
        double clawPos = 0;


        //FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        while(opModeIsActive()){
            telemetry.update();

            el.setPower(-gamepad1.right_stick_y);
            er.setPower(-gamepad1.right_stick_y);
            sl.setPower(-gamepad1.right_stick_x);
            sr.setPower(gamepad1.right_stick_x);
            if (gamepad1.x) {
                rollPwr = 1;
            } else if (gamepad1.y) {
                rollPwr = -1;
            }
            roll.setPosition(rollPwr);

            if (dlOVERRIDE == 0 && drOVERRIDE == 0) {
                if (gamepad1.right_bumper && !rbDebounce) {
                    clawState = !clawState;
                    rbDebounce = true;
                } else if (!gamepad1.right_bumper) {
                    rbDebounce = false;
                }
                if (clawState) {
                    drPos = 0.2;
                    dlPos = 0.2;
                } else {
                    drPos = 0;
                    dlPos = 0;
                }
                if (gamepad1.a) {
                    clawPos = Range.clip(clawPos+0.01,0,1);
                } else if (gamepad1.b) {
                    clawPos = Range.clip(clawPos-0.01,0,1);
                }
                drPos += 0.5+clawPos;
                dlPos += 0.5-clawPos;
                dr.setPosition(drPos);
                dl.setPosition(dlPos);
            } else {
                dr.setPosition(drOVERRIDE);
                dl.setPosition(dlOVERRIDE);
            }
            telemetry.addData("drPos", drPos);
            telemetry.addData("dlPos", dlPos);


            double xPwr = gamepad1.left_stick_x;
            double yPwr = gamepad1.left_stick_y;
            double desiredPower = Range.scale(sqrt(xPwr*xPwr+yPwr*yPwr), 0, sqrt(2), 0, 1); //convert the diagonal of the sticks to magnitude
            double turn;


            boolean turbo = (gamepad1.left_stick_button || gamepad1.right_stick_button);
            float rotate = (gamepad1.right_trigger - gamepad1.left_trigger);//controller turn input
            if (turbo){
                desiredHeading += rotate*-4;
            } else {
                desiredHeading += rotate*-1;
            }



            if (transientSpeed >= desiredPower + 0.001) {
                //transientSpeed -= (abs(desiredPower - transientSpeed))/5;
                transientSpeed -= (transientSpeed - desiredPower)*rampSpeed;
            } else if (transientSpeed < desiredPower - 0.001) {
                transientSpeed += (desiredPower - transientSpeed)*rampSpeed;
            }


            double direction = (toDegrees(atan2(gamepad1.left_stick_x, gamepad1.left_stick_y)));

            //All the field centric stuff is right here, copy/pasted from the last code but never really tested
            //I HOPE THIS WORKS, CUZ I DIDNT VERIFY IT
            //I guess we need a thing to track the jump from 360 degrees to zero and vice versa, cuz there probably isn't any negative degrees
            double lastHeading = heading;
            angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //poll IMU
            heading = angles.firstAngle; //extract heading
            telemetry.addData("Heading:", heading);

            //the below direction tracking code is inspired by VCSC
            //I could make this dynamic, where the controller tracks the time between calls and calculates the max possible rotation in that time
            if (heading - lastHeading > 270) {
                timesAcrossZero--;
            } else if (heading - lastHeading < -270){
                timesAcrossZero++;
            }
            fixedHeading = heading + (timesAcrossZero * 360);
            if (gamepad1.dpad_left && !dpadDebounce) {
                fieldCentric = !fieldCentric;
                dpadDebounce = true;
            } else if (!gamepad1.dpad_left) {
                dpadDebounce = false;
            }
            if (fieldCentric) { //when field centric, rotate control with IMU
                direction = ((direction - fixedHeading) + 360) % 360; //augment joystick command with IMU direction, keeping within the range of 360
            }

            if (rotate == 0 && justTurned) {
                desiredHeading = fixedHeading;
            }
            justTurned = rotate != 0;

            turn = Range.clip(Range.scale(fixedHeading - desiredHeading, -headingCorrectionRange, headingCorrectionRange, -1, 1), -1, 1);



            //the constants in the ramping functions below are tunable!
            if (transientTurn > turn + 0.001) {
                transientTurn = turn;
            } else if (transientTurn < turn - 0.001) {
                transientTurn += (turn - transientTurn)*turnRamp;
            }

            //Mecanum wheel diagonals spin the same direction, except when turning
            //set the powers of rf and lf, and lr and lf
            rightDiagPwr = (Range.clip((1.4*cos(toRadians(direction) + 3.1415/4)), -1, 1))*transientSpeed;
            leftDiagPwr = (Range.clip((1.4*cos(toRadians(direction) - 3.1415/4)), -1, 1))*transientSpeed;


            //what it does is scales down the drive power depending on how much it wants to turn, then adds turn to the scaled value
            //recently added is when the voltage dips
            double RFPwr = ((leftDiagPwr*(1-Range.clip(abs(transientTurn),0,turnOverride)))+transientTurn);
            double lfPwr = ((rightDiagPwr*(1-Range.clip(abs(transientTurn),0,turnOverride)))-transientTurn);
            double lrPwr = ((leftDiagPwr*(1-Range.clip(abs(transientTurn),0,turnOverride)))-transientTurn);
            double RRPwr = ((rightDiagPwr*(1-Range.clip(abs(transientTurn),0,turnOverride)))+transientTurn);

            //Alfly final values to motors, with power and turbo modifiers
            if (!turbo) {
                RFPwr /= 2;
                lfPwr /= 2;
                lrPwr /= 2;
                RRPwr /= 2;
                justTurbo = false;
            } else {
                if (!justTurbo) {
                    desiredHeading = heading + rotate;
                }
                justTurbo = true;
            }
            rf.setPower(RFPwr);
            lf.setPower(lfPwr);
            lr.setPower(lrPwr);
            rr.setPower(RRPwr);

            telemetry.addData("direction", direction);
            telemetry.addData("desiredPower", desiredPower);


        }

    }
}