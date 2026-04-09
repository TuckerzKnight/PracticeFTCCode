package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AutoConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .forwardZeroPowerAcceleration(-42.1919355)
        .lateralZeroPowerAcceleration(-209.02505251);

    public static MecanumConstants driveConstants = new MecanumConstants()
        .maxPower(0.3)
        .rightFrontMotorName("frontRight")
        .rightRearMotorName("backRight")
        .leftRearMotorName("backLeft")
        .leftFrontMotorName("frontLeft")
        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
        .xVelocity(74.090772)
        .yVelocity(56.582035);
    public static PinpointConstants localizerConstants = new PinpointConstants()
        .forwardPodY(2.5)
        .strafePodX(7)
        .distanceUnit(DistanceUnit.INCH)
        .hardwareMapName("pinpoint")
        .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
        .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
        .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
            .pathConstraints(pathConstraints)
            .mecanumDrivetrain(driveConstants)
            .pinpointLocalizer(localizerConstants)
            .build();
    }
}
