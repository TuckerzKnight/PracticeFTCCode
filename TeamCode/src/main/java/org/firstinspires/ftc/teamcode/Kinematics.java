package org.firstinspires.ftc.teamcode;

//so this doesn't have anything in it yet
//but when we start doing autos, put basic encoder to coordinate system, or imu to coordinate, then we can call this class
//to get robot pose
//hmm, I need some sort of unit testing big time

import static java.lang.Math.acos;
import static java.lang.Math.asin;
import static java.lang.Math.atan;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Kinematics {
    public static double fieldSize = 141*25.4; //approximate field length
    public static double robotwidth = 327.025;
    public static double robotLength = 12.5*25.4;
    public static double robotWheelBase = 224; //width

    public static double startingPos = 47.25*25.4; //distance from second nearest wall

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
    static public double elbowRatio = (68.15*1.4);//I think 20t:28
    static public double shoulderRatio = 143.05;
    static public double armHeight = 104;//axle height above ground with 5mm of tile sink, feel free to adjust

    public static double [] endPos = {0,0,0,0}; // index zero refers to x, 1 to y, 2 to pitch

    public static double [] buffer = {0,0,0,0};



    public void encoderUpdate () {

        //kinematics here

        //for now I do this:
        //the difference between the average power of each side should hopefully determine the turning,
        //the average motor power should hopefully determine forward/backward,
        //the difference between front and back should hopefully determine strafing
        turn = (((Base.rfPos - rfLastPos) + (Base.rrPos - rrLastPos))/2.0) - (((Base.lfPos - lfLastPos) + (Base.lrPos - lrLastPos))/2.0);
        straight = ((Base.rfPos - rfLastPos) + (Base.rrPos - rrLastPos) + (Base.lfPos - lfLastPos) + (Base.lrPos - lrLastPos))/4.0;
        strafe = (((Base.rfPos - rfLastPos) - (Base.rrPos - rrLastPos)) - ((Base.lfPos - lfLastPos) - (Base.lrPos - lrLastPos)))/2.0;

        //this converts the turn ticks to a distance the wheels moved to turn, and uses that to hopefully calculate the angle based on
        //the radius of the turn. Perhaps it needs a term to factor in variable turn radius. Could I plot this on Desmos?
        heading += ((turn*distancePerRevolution/ticksPerRevolution)/(robotWheelBase*6.283))%360;
        //the idea of these is to take the sinusoid of the angle times the hypotenuse to find a side
        Y -= sin(heading)*straight*distancePerRevolution/ticksPerRevolution;
        X += cos(heading)*straight*distancePerRevolution/ticksPerRevolution;
        Y -= cos(heading)*strafe*strafingEfficiency*distancePerRevolution/ticksPerRevolution;
        X += sin(heading)*strafe*strafingEfficiency*distancePerRevolution/ticksPerRevolution;
        rrLastPos = (int)Base.rrPos;
        rfLastPos = (int)Base.rfPos;
        lrLastPos = (int)Base.lrPos;
        lfLastPos = (int)Base.lfPos;


    }

    //make sure to call this before using any of the position presets, to format them to relative ticks rather than end pose
    public void ArmKineInit (Telemetry telemetry) {
        //Here, we calculate the required motor positions to place the claw at a specified point and rotation, which loads the results onto
        //a buffer array that can then be copied over to the base class state presets for use throughout an opmode
        backtakeIK(Base.backtakePos[0],Base.backtakePos[1], telemetry); //compute
        Base.backtakePos[0] = buffer[1]; //save computed values
        Base.backtakePos[1] = buffer[2];
        Base.backtakePos[2] = buffer[3];
        backtakeIK(Base.stackPos[0],Base.stackPos[1], telemetry);
        Base.stackPos[0] = buffer[1]; //rear intake and stack intake are nearly identical, except stack can iterate position
        Base.stackPos[1] = buffer[2];
        Base.stackPos[2] = buffer[3];
        backtakeIK(Base.lowerStackPos[0],Base.lowerStackPos[1], telemetry);
        Base.lowerStackPos[0] = buffer[1];
        Base.lowerStackPos[1] = buffer[2];
        Base.lowerStackPos[2] = buffer[3];
        armIK(Base.depositPos[0],Base.depositPos[1],Base.depositPos[2],telemetry);
        Base.depositPos[0] = buffer[0];
        Base.depositPos[1] = buffer[1]; //depositing requires another input, because it makes use of the shoulder motors, adding a DOF
        Base.depositPos[2] = buffer[2];
        Base.depositPos[3] = buffer[3];
        armIK(Base.backDepositPos[0],Base.backDepositPos[1],Base.backDepositPos[2], telemetry);
        Base.backDepositPos[0] = buffer[0];
        Base.backDepositPos[1] = buffer[1]; //This is an alternative deposit if you're facing the opposite direction
        Base.backDepositPos[2] = buffer[2];
        Base.backDepositPos[3] = buffer[3];
    }
    public void ArmKineUpdate(){
        //convert arm positioning to degrees
        Base.jointPositions[0] = (((Base.jointPositions[0])/shoulderRatio)*(ticksPerRevolution/360));
        Base.jointPositions[1] = ((((Base.jointPositions[1])/elbowRatio)*(ticksPerRevolution/360)))-Base.jointPositions[0];//due to V4b, subtract base

        //theta could be treated as the pitch angle of the robot, which should always be zero, and depth is the number of links to
        //compute, from 0-n. This simply updates the endPos[] array
        computeLink(X, Y, 0, 2);
    }

    //recursively descend through each links offsets to a passed-in number of links, in this case 2 (counting starts at zero)
    public void computeLink(double x, double y, double theta, int depth) {
        endPos[0] = (cos(Base.jointPositions[depth]) * Base.links[depth]) + x;
        endPos[1] = (sin(Base.jointPositions[depth]) * Base.links[depth]) + y;
        endPos[2] = theta + Base.jointPositions[depth];
        if (depth > 0) {
            computeLink(endPos[0], endPos[1], endPos[2],(depth-1));
        }
    }

    //Find required angles for a configuration, and verify they are within hardware limits
    //Please visit the associated Desmos graph, is very helpful: https://www.desmos.com/calculator/unvhc2shut
    //sample input positions: -100, 450, 53
    //links values as of 3/29/24: 319, 269.19, 112
    static public boolean armIK (double x, double y, double theta, Telemetry telemetry) {

        double endLength = Base.links[Base.links.length - 1];//112
        x += cos(theta) * endLength;//~60; since the end link (claw) position is fully predefined, we offset the target coords to where the claw starts
        y += (sin(theta) * endLength) - armHeight;//~-8; since the arm axle is above the ground, apply offset
        double endDist = sqrt(x*x+y*y);//with sample values, ~460
        //triangulate the angle required to create a triangle with points base and end, by calculating the angle from the target and subtracting
        //the shoulder triangulation angle from that
        //~34º
        double shoulderTheta = acos(((Base.links[1] * Base.links[1]) - (endDist*endDist) - (Base.links[0] * Base.links[0])) / (-2 * Base.links[0] * endDist));
        double sumTheta = atan(y/x);//angle to end, inverts across y axis tho
        double elbowTheta;
        buffer[0] = sumTheta - shoulderTheta; //global shoulder angle
        if (x < 0) {
            elbowTheta = Base.links[0]*-cos(sumTheta-shoulderTheta);//relative angle
            buffer[1] = -(shoulderTheta + elbowTheta - 3.1415 - sumTheta); //global elbow angle
        } else {
            elbowTheta = Base.links[0]*cos(sumTheta-shoulderTheta);//relative angle
            buffer[1] = -(shoulderTheta+elbowTheta-sumTheta);//global angle
        }
        buffer[2] = ((theta - buffer[1])%180); //find global wrist angle
        buffer[1] = (buffer[1]+(buffer[0]/elbowRatio));//offset global position to motor perceived angle

        if (buffer[3] > 0 && buffer[3] < Base.rollTravel) {
            buffer[3] = buffer[3]/Base.rollTravel;//this is in servo output range
        } else {
            buffer[3] = 0;//roll to home
            Base.errorCode = 2;
        }

        //if the target is too far away, exit
        if(endDist > Base.links[0]+Base.links[1]+Base.links[2]) {
            telemetry.addLine("Target out of range");
            return(false);
        } else if (y < (-armHeight)) {
            telemetry.addLine("Target in ground");
            return(false);
        } else if ((theta > Base.wristTravel/2) || (theta < 0)) {
            telemetry.addLine("Wrist out of bounds");
            return(false);
        } else {
            //No issues detected
            return(true);
        }
    }


    //for backtaking behind the robot (for weight balance reasons), IK is different as only the elbow and wrist will be actuated
    public boolean backtakeIK (double y, double theta, Telemetry telemetry) {
        if (y >= 0) {
            //code here
            y += sin(theta)*Base.links[2];
            //the code below assumes the forearm at rest is at 0 degrees, so ensure arm is initialized to the real angle on init
            //either run armKineUpdate before this, or make sure it never runs and apply ticks to degrees here
            //we're also gonna assume that shoulder is at zero, bc otherwise we have to convert to global coords
            buffer[1] = (180-asin((y-armHeight)/Base.links[1]));
            buffer[2] = (buffer[1] - 180) + 45;//ok this increases as the elbow angle increases, but when elbow is
                                                                        //exactly extended, it will be at 45 degrees
            buffer[3] = 1;//roll buffer
            return(true);
        }
        telemetry.addLine("backtake target too low");
        return(false);
    }
}
