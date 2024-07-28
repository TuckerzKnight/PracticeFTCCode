package org.firstinspires.ftc.teamcode;

import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivebase {

    private IMU imu;

    public static double [] robotStats = {5,6,7,8,9};
    double direction = 0;
    double power = 0;
    VoltageSensor myControlHubVoltageSensor;
    public static double batteryMax = 14;
    public static double batteryMin = 9;
    public static double heading;
    public static double desiredHeading;
    public static double fixedHeading;
    public static double timesAcrossZero;
    public static double lastHeading;
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    double batteryAverage = (batteryMax+batteryMin)/2;
    void init (HardwareMap hMap) {
        imu = hMap.get(IMU.class, "imu");
        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hMap.get(DcMotorEx.class, "frontLeft");
        backLeftMotor = hMap.get(DcMotorEx.class, "rearLeft");
        frontRightMotor = hMap.get(DcMotorEx.class, "frontRight");
        backRightMotor = hMap.get(DcMotorEx.class, "rearRight");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();
        desiredHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        myControlHubVoltageSensor = hMap.get(VoltageSensor.class, "Control Hub");
    }
    public void update(Gamepad gamepad1, HardwareMap hMap) {
        //make sure you don't poll the IMU anywhere else, either just reference this variable or define IMU readings in main opmode or class
        if (gamepad1.dpad_left) {
            imu.resetYaw();
        }
        lastHeading = heading;
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        if ((heading - lastHeading) > 3.14159) {
            timesAcrossZero--;
        } else if ((heading - lastHeading) < -3.14159) {
            timesAcrossZero++;
        }
        fixedHeading = (heading + timesAcrossZero*6.28318);
        robotStats[0] = heading;
        double powerRegulation = batteryAverage/myControlHubVoltageSensor.getVoltage();

        batteryMax = max(myControlHubVoltageSensor.getVoltage(),batteryMax);
        batteryMin = min(myControlHubVoltageSensor.getVoltage(),batteryMin);

        desiredHeading += ((gamepad1.left_trigger - gamepad1.right_trigger)/25);//Here's a turn speed constant
        direction = (atan2(gamepad1.left_stick_y, gamepad1.left_stick_x)+(3.14159/2));
        robotStats[1] = direction;
        power = sqrt((gamepad1.left_stick_y* gamepad1.left_stick_y)+(gamepad1.left_stick_x* gamepad1.left_stick_x));
        double straight = power*cos((-heading)-direction);
        double strafe = power*sin((-heading)-direction)*1.1;
        double turn = ((fixedHeading-desiredHeading)/3.14159);
        double denominator = max(abs(straight) + abs(strafe) + abs(turn), 1);
        frontRightMotor.setPower((straight - turn + strafe)*powerRegulation/denominator);
        frontLeftMotor.setPower((straight + turn - strafe)*powerRegulation/denominator);
        backRightMotor.setPower((straight - turn - strafe)*powerRegulation/denominator);
        backLeftMotor.setPower((straight + turn + strafe)*powerRegulation/denominator);
    }
}
