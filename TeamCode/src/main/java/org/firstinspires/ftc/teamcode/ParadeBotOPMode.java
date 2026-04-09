package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import kotlinx.coroutines.internal.OpDescriptor;

@TeleOp(name ="candy")
public class ParadeBotOPMode extends OpMode {
    DcMotor leftMotor, rightMotor, BigWheelMotor;
    @Override
    public void init() {
        //leftMotor = hardwareMap.get();
    }

    @Override
    public void loop() {

    }
}
