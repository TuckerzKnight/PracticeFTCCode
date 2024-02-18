package org.firstinspires.ftc.teamcode;

//so this doesn't have anything in it yet
//but when we start doing autos, put basic encoder to coordinate system, or imu to coordinate, then we can call this class
//to get robot pose
//hmm, I need some sort of unit testing big time

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Localization {
    public static double fieldSize = 141*25.4; //approximate field length
    public static double robotwidth = 327.025;
    public static double robotLength = 12.5*25.4;
    public static double robotWheelBase = 224; //width

    public static double startingPos = 47.25*25.4; //distance from second nearest wall

    DcMotorEx rf, rr, lf, lr;

    int rfTicks = 0, rrTicks = 0, lfTicks = 0, lrTicks = 0;
    int rfLastPos = 0, rrLastPos = 0, lfLastPos = 0, lrLastPos = 0;

    double ticksPerRevolution = 28*2.89*5.23; //on a 3/1 and 5/1 UltraPlanetary setup
    double distancePerRevolution = 75*3.1415;//distance in mm

    public static double strafingEfficiency = 0.9; //google says strafing goes about this fast relative to straight driving


    double turn = 0;
    double straight = 0;
    double strafe = 0;

    static public double X = /*robotLength/2*/0;
    static public double Y = /*fieldSize-startingPos*/0;
    static public double heading;


    public void locaInit (HardwareMap hMap) {
        rf = hMap.get(DcMotorEx.class, "frontright");
        rr = hMap.get(DcMotorEx.class, "rearright");
        lf = hMap.get(DcMotorEx.class, "frontleft");
        lr = hMap.get(DcMotorEx.class, "rearleft");


    }

    public void encoderUpdate (Telemetry telemetry) {
        rfTicks = -rf.getCurrentPosition();
        rrTicks = -rr.getCurrentPosition();
        lfTicks = -lf.getCurrentPosition();//inverting these, since the motors face the opposite direction
        lrTicks = -lr.getCurrentPosition();

        //kinematics here


        //for now I do this:
        //Theres gonna be a lot of signs wrong until I figure all that out
        //the difference between the average power of each side should hopefully determine the turning,
        //the average motor power should hopefully determine forward/backward,
        //the difference between front and back should hopefully determine strafing
        turn = (((rfTicks - rfLastPos) + (rrTicks - rrLastPos))/2.0) - (((lfTicks - lfLastPos) + (lrTicks - lrLastPos))/2.0);
        straight = ((rfTicks - rfLastPos) + (rrTicks - rrLastPos) + (lfTicks - lfLastPos) + (lrTicks - lrLastPos))/4.0;
        strafe = (((rfTicks - rfLastPos) - (rrTicks - rrLastPos)) - ((lfTicks - lfLastPos) - (lrTicks - lrLastPos)))/2.0;

        //this converts the turn ticks to a distance the wheels moved to turn, and uses that to hopefully calculate the angle based on
        //the radius of the turn. Perhaps it needs a term to factor in variable turn radius. Could I plot this on Desmos?
        heading += ((turn*distancePerRevolution/ticksPerRevolution)/(robotWheelBase*6.283))%360;
        //the idea of these is to take the sinusoid of the angle times the hypotenuse to find a side
        Y -= sin(heading)*straight*distancePerRevolution/ticksPerRevolution;
        X += cos(heading)*straight*distancePerRevolution/ticksPerRevolution;
        Y -= cos(heading)*strafe*strafingEfficiency*distancePerRevolution/ticksPerRevolution;
        X += sin(heading)*strafe*strafingEfficiency*distancePerRevolution/ticksPerRevolution;
        rrLastPos = rrTicks;
        rfLastPos = rfTicks;
        lrLastPos = lrTicks;
        lfLastPos = lfTicks;
        telemetry.addData("rfTicks", rfTicks);
        telemetry.addData("rrTicks", rrTicks);
        telemetry.addData("lfTicks", lfTicks);
        telemetry.addData("lrTicks", lrTicks);
        telemetry.addData("rfLastPos",rfLastPos);
        telemetry.addData("straight", straight);
        telemetry.addData("turn", turn);
        telemetry.addData("strafe", strafe);


    }
}
