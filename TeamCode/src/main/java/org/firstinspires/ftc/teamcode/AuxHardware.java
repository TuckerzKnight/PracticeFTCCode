//This code is deprecated. Please view Base.java to find management code

//Code for controlling non-drive motors should go here

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AuxHardware {

    //define any and all motors for functions here
    DcMotorEx motor1, motor2, motor3, motor4;

    //define all servos here, as many as any one feature uses. perhaps unambiguity would reduce naming collisions
    Servo servo1, servo2, servo3, servo4, servo5, servo6;

    double armTicksPerDegrees;

    int armPosition;

    int armMin;
    int armMax;

    int wrist = 0;
    int wristMin = 0;
    int wristMax = 90;

    void simpleArmInit (HardwareMap hMap, double encoderTicksPerDegrees, int min, int max) {

        //designate hardware to a specific role
        motor1 = hMap.get(DcMotorEx.class, "arm");
        servo1 = hMap.get(Servo.class, "claw");
        servo2 = hMap.get(Servo.class,"wrist");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setTargetPosition((int)(min*armTicksPerDegrees));
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setPower(0.1);

        armTicksPerDegrees = encoderTicksPerDegrees;
        armPosition = /*-min*/(int)armTicksPerDegrees; //assuming the arm is resting on the ground, set zero to 2 degrees above
        armMin = min;
        armMax = max;
        servo2.setPosition(0);
    }
    //add ramping?
    void simpleArmMove (double degrees, double speed) {
        //ensure you can't tell the motor to go beyond its range of motion
        /*if (armPosition + degrees < armMin) {
            degrees = -armPosition;
        } else if (armPosition + degrees > armMax) {
            degrees = armMax - armPosition;
        }*/
        armPosition = (int)(degrees*armTicksPerDegrees);
        motor1.setPower(speed);
        motor1.setTargetPosition(armPosition);

        //this is where the problem is
        servo2.setPosition(Range.clip(Range.scale(degrees, 0, armMax, 0, 1)-Range.scale(wrist, wristMin, wristMax, -1, 1), 0, 1));

    }

    //breakbeam sensors? color sensors?
    void clawOpen () {
        servo1.setPosition(0.2);
    }

    void clawClose () {
        servo1.setPosition(0.7);
    }

    void wristPosition (int degrees) {
        //ensure you can't tell the motor to go beyond its range of motion
        if (wrist + degrees < wristMin) {
            degrees = -wrist;
        } else if (wrist + degrees > wristMax) {
            degrees = wristMax - armPosition;
        }
        servo2.setPosition(Range.scale(degrees, 0, 360, 0, 1)-armPosition);
    }

}
