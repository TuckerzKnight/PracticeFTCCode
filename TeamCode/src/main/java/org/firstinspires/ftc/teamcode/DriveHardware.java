//The drive update functions are dependant upon each joystick range being from -1 - 1, I don't know if that is the case
//the mecanum drive thingummy has a toRadians(), and I don't know that it is used at this point, and it probably will cause problems


package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Config
public class DriveHardware {
    //define motors as existing
    DcMotorEx lr, rf, lf, rr;

    private IMU imu;

    public static double heading;

    public static double desiredHeading;

    static public int timesAcrossZero;

    double fixedHeading;

    Orientation angles;

    //define random other garbage for remembering gamepad state
    boolean lastBumper = false;
    boolean currentBumper = false;
    boolean bumperDown = true;

    boolean resetDown = false;

    boolean justTurned = false;

    double transientSpeed;

    double  transientTurn;

    public static double rampSpeed = 0.2; //Rate in percentage at which function ramps to speed

    public static double turnOverride = 0.75; //at one turning can completely override drive, at zero turning has no effect

    public static double turnRamp = 0.5; //this can be adjusted to slow down or speed up the turning acceleration

    public static double headingCorrectionRange = 45; //this scales the reaction of the heading correction-
    // smaller values encourage snappy reaction, but larger values decrease overshoot

    double leftDiagPwr;
    double rightDiagPwr;

    //yo imma try PID
    //so as I understand it, p is multiplied against error for a base output, I increases when error stays for a while to increase
    //response, and d keeps things from overcompensating
    public static double Kp = 1;
    public static double Ki = 0.0;
    public static double Kd = 1;
    double lastError = 0;
    double integralSum = 0;

    public static boolean fieldCentric = true;
    boolean justTurbo = false;

    ElapsedTime PIDTimer = new ElapsedTime();

    //This function assigns hardware malfing and motor behavior
    public void tankDriveInit(final HardwareMap hMap){
        //Certainly none of this was stolen

        //use hardware specific inputs to calculate what the encoders need to see
        //if you're using encoders, this is necessary, with arguments of  ints ticksPerRevolution, driveReductionRatio, wheelDia in the function definition
        //double ticksPerSpeed = ((ticksPerRevolution*driveReductionRatio*wheelDia*3.14)/12*speed);



        //Here you can choose to brake or coast
        DcMotor.ZeroPowerBehavior zeroPower = DcMotor.ZeroPowerBehavior.BRAKE;

        //I haven't learned enough yet, but VCSC doesn't use the below and IDE doesn't accept it
        //lr = hardwareMap.get(DcMotorEx.class, "Left Front")

        //*dhuughgh* it wurks :P
        rr = hMap.get(DcMotorEx.class, "frontLeft");
        lf = hMap.get(DcMotorEx.class, "frontRight");
        rf = hMap.get(DcMotorEx.class, "rearLeft");
        lr = hMap.get(DcMotorEx.class, "rearRight");

        //Here you can edit the individual motor headings
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        lr.setZeroPowerBehavior(zeroPower);
        rf.setZeroPowerBehavior(zeroPower);
        lf.setZeroPowerBehavior(zeroPower);
        rr.setZeroPowerBehavior(zeroPower);

        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //OK here, you should have gamepad control and stuff
    void tankDriveUpdate (double turn, float power, boolean turbo) {
        //Create a small dead zone in the center of the sticks travel
        if (turn < 0.08 && turn > -0.08) {turn = 0;}
        if (power < 0.08 && power > -0.08) {power = 0;}

        //square the stick output to allow greater low-end resolution
        //thanks to Dallin for the ideaZs
        boolean isNegative = power < 0;
        //woah I looked through the warnigns and it suggested the above logic
        //I tried to do that, but had done it wrong and couldn't figure it out

        power = power*power;
        //make sure the proper sign is retained || it sounds like the y joystick does go through to negative, if not this all is wrong
        if (isNegative){
            power = power * -1;
        }

        isNegative = turn < 0;
        if (turn != 0) {
            turn = (turn*turn/-2)-0;//this constant here is to compensate for drift
        } else {
            turn = (turn*turn/-2);
        }

        if (isNegative) {
            turn = turn*-1;
        }
        if (!turbo) {
            turn = turn/2;
            power = power/2;
        }

        //hopefully this does the tank drive mixing right
        lr.setPower(Range.clip(power-turn, -1, 1));
        lf.setPower(Range.clip(power-turn, -1, 1));
        rf.setPower(Range.clip(power+turn, -1, 1));
        rr.setPower(Range.clip(power+turn, -1, 1));


    }
    public void mecanumDriveInit(final HardwareMap hMap){
        //Most definitely none of this was stolen
        //hardware map IMU
        imu = hMap.get(IMU.class, "imu");
        angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //poll IMU
        heading = Range.scale(angles.firstAngle, 0, 360, -1, 1); //extract heading
        imu.resetYaw();

        timesAcrossZero = 0;
        transientSpeed = 0;
        desiredHeading = 0;
    }


    //This function is designed solely for teleop useage, I hope once everything is tuned I'll make a function to directly
    //pass in direction and power and turn
    void mecanumDriveUpdateV2 (Gamepad gamepad, Telemetry telemetry) {
        double xPwr;
        //get x value from active thumbstick, or if both are active, average them
        if (Base.state == Base.State.DEPOSIT || Base.state == Base.State.BACKPUT) {
            xPwr = gamepad.left_stick_x;
        } else {
            if (gamepad.left_stick_x != 0) {
                if (gamepad.right_stick_x == 0) {
                    xPwr = gamepad.left_stick_x;
                } else {
                    xPwr = (gamepad.left_stick_x + gamepad.right_stick_x) / 2;
                }
            } else {
                xPwr = gamepad.right_stick_x;
            }
        }
        double yPwr = 0;
        //disable right joystick y when depositing, its needed for height adjustment
        if (Base.state == Base.State.DEPOSIT || Base.state == Base.State.BACKPUT) {
            yPwr = gamepad.left_stick_y;
        } else {
            //get y power from active thumbstick(s)
            if (gamepad.left_stick_y != 0) {
                if (gamepad.right_stick_y == 0) {
                    yPwr = gamepad.left_stick_y;
                } else {
                    yPwr = (gamepad.left_stick_y + gamepad.right_stick_y)/2;
                }
            } else {
                yPwr = gamepad.right_stick_y;
            }
        }
        double desiredPower = Range.scale(sqrt(xPwr*xPwr+yPwr*yPwr), 0, sqrt(2), 0, 1); //convert the diagonal of the sticks to magnitude
        double turn;


        boolean turbo = (gamepad.left_stick_button || gamepad.right_stick_button);
        float rotate = (gamepad.right_trigger - gamepad.left_trigger);//controller turn input
        if (turbo){
            desiredHeading += rotate*-4;
        } else {
            desiredHeading += rotate*-2;
        }



        if (transientSpeed >= desiredPower + 0.001) {
            //transientSpeed -= (abs(desiredPower - transientSpeed))/5;
            transientSpeed -= (transientSpeed - desiredPower)*rampSpeed;
        } else if (transientSpeed < desiredPower - 0.001) {
            transientSpeed += (desiredPower - transientSpeed)*rampSpeed;
        }

        double direction;
        //find out which sticks are active, and use those
        if (Base.state == Base.State.DEPOSIT || Base.state == Base.State.BACKPUT) {
            direction = (toDegrees(atan2(gamepad.left_stick_x, gamepad.left_stick_y))); //convert sticks to angle
        } else {
            if ((gamepad.left_stick_x != 0 || gamepad.left_stick_y != 0) && (gamepad.right_stick_x == 0 && gamepad.right_stick_y == 0)) {
                direction = (toDegrees(atan2(gamepad.left_stick_x, gamepad.left_stick_y))); //convert sticks to angle
            } else if ((gamepad.right_stick_x != 0 || gamepad.right_stick_y != 0) && (gamepad.left_stick_x == 0 && gamepad.left_stick_y == 0)) {
                direction = (toDegrees(atan2(gamepad.right_stick_x, gamepad.right_stick_y))); //convert sticks to angle
            } else {
                direction = (toDegrees((atan2(gamepad.left_stick_x, gamepad.left_stick_y) + atan2(gamepad.right_stick_x, gamepad.right_stick_y)) / 2)); //convert sticks to angle
            }
        }

        //All the field centric stuff is right here, copy/pasted from the last code but never really tested
        //I HOPE THIS WORKS, CUZ I DIDNT VERIFY IT
        //I guess we need a thing to track the jump from 360 degrees to zero and vice versa, cuz there probably isn't any negative degrees
        double lastHeading = heading;
        angles = imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //poll IMU
        heading = angles.firstAngle; //extract heading

        //apply alliance side offset
        /*if (Base.alliance) {
            heading = (heading +90) % 180;
        } else {
            heading = (heading - 90) % 180;
        }*/
        telemetry.addData("Heading:", heading);

        //the below direction tracking code is inspired by VCSC
        //I could make this dynamic, where the controller tracks the time between calls and calculates the max possible rotation in that time
        if (heading - lastHeading > 270) {
            timesAcrossZero--;
        } else if (heading - lastHeading < -270){
            timesAcrossZero++;
        }

        //make sure to apply alliance offset in reverse to this, so that changing alliance doesn't affect heading correction
        /*if (Base.alliance) {
            fixedHeading = heading + (timesAcrossZero * 360) - 90;
        } else {
            fixedHeading = heading + (timesAcrossZero * 360) + 90;
        }*/fixedHeading = heading + (timesAcrossZero * 360);
        if (fieldCentric) { //when field centric, rotate control with IMU
            direction = ((direction - fixedHeading) + 360) % 360; //augment joystick command with IMU direction, keeping within the range of 360
        }

        if (rotate == 0 && justTurned) {
            desiredHeading = fixedHeading;
        }
        justTurned = rotate != 0;

        //the below PID stuff doesn't seem to work, disalfointingly

        /*double error = fixedHeading - desiredHeading;

        double derivative = (error-lastError)/PIDTimer.seconds();

        integralSum = integralSum+(error*PIDTimer.seconds());

        turn = Range.clip(((Kp * error) + (Ki * integralSum) + (Kd * derivative))*-1, -1, 1);*/
        turn = Range.clip(Range.scale(fixedHeading - desiredHeading, -headingCorrectionRange, headingCorrectionRange, -1, 1), -1, 1);

        /*lastError = error;
        //PIDTimer.reset();*/


        //the constants in the ramping functions below are tunable!
        if (transientTurn > turn + 0.001) {
            transientTurn = turn;
        } else if (transientTurn < turn - 0.001) {
            transientTurn += (turn - transientTurn)*turnRamp;
        }

        //Mecanum wheel diagonals spin the same direction, except when turning
        //set the powers of rf and lf, and lr and lf
        /*rightDiagPwr = (Range.clip((1.4*cos(toRadians(direction) + 3.1415/4)), -1, 1))*transientSpeed;
        leftDiagPwr = (Range.clip((1.4*cos(toRadians(direction) - 3.1415/4)), -1, 1))*transientSpeed;


        //what it does is scales down the drive power depending on how much it wants to turn, then adds turn to the scaled value
        //recently added is when the voltage dips
        double RFPwr = ((leftDiagPwr*(1-Range.clip(abs(transientTurn),0,turnOverride)))+transientTurn)*Base.powerMultiplier;
        double lfPwr = ((rightDiagPwr*(1-Range.clip(abs(transientTurn),0,turnOverride)))-transientTurn)*Base.powerMultiplier;
        double lrPwr = ((leftDiagPwr*(1-Range.clip(abs(transientTurn),0,turnOverride)))-transientTurn)*Base.powerMultiplier;
        double RRPwr = ((rightDiagPwr*(1-Range.clip(abs(transientTurn),0,turnOverride)))+transientTurn)*Base.powerMultiplier;*/

        //Thanks to Gm0 for this code
        double straight = transientSpeed*cos((-heading)-direction);
        double strafe = transientSpeed*sin((-heading)-direction)*1.1;
        double denominator = Double.max(abs(straight) + abs(strafe) + abs(turn), 1);
        double RFPwr = (straight - turn + strafe)/denominator;
        double lfPwr = (straight + turn - strafe)/denominator;
        double lrPwr = (straight - turn - strafe)/denominator;
        double RRPwr = (straight + turn + strafe)/denominator;

        //Alfly final values to motors, with power and turbo modifiers
        if (turbo) {
            RFPwr /= 2;
            lfPwr /= 2;
            lrPwr /= 2;
            RRPwr /= 2;
            justTurbo = false;
        } else {
            if (!justTurbo) {
                desiredHeading = fixedHeading + rotate;
            }
            justTurbo = true;
        }
        //send values to base class for output
        Base.rfPwr = (RFPwr);
        Base.lfPwr = (lfPwr);
        Base.lrPwr = (lrPwr);
        Base.rrPwr = (RRPwr);

        telemetry.addData("direction", direction);
        telemetry.addData("desiredPower", desiredPower);
    }
}
