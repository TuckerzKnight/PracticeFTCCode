//The source of my SDK knowledge: https://ftc-code.gitbook.io/tech-toolbox/getting-started/motors-and-encoders
//How to connect to on bot java: https://ftc-docs.firstinspires.org/en/latest/programming_resources/shared/program_and_manage_network/Connecting-a-Laptop-to-the-Program-%26-Manage-Network.html

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DriveHardware;

@TeleOp(name = "TankTeleop")
public class PracticeLinearTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization Code Goes Here

        //cool
        DriveHardware tankDrive;
        tankDrive = new DriveHardware();
        tankDrive.tankDriveInit(hardwareMap);

        double encoderTicksPerRevolution = 28/*125*0.72/180*/;

        /*AuxHardware arm;
        arm = new AuxHardware();
        arm.simpleArmInit(hardwareMap, encoderTicksPerRevolution, 2, 130);

        double armPosition = 0;
        double armSpeed = 1;
        boolean clawState = false;
        boolean Lbumper = false;
        double leftXMin = 0;*/


        waitForStart();
        while(opModeIsActive()){ //while loop for when program is active
            //Code repeated during teleop goes here
            //Analogous to loop() method in OpMode

            //run a tank drive
            tankDrive.tankDriveUpdate(gamepad1.right_stick_x, gamepad1.right_stick_y, gamepad1.right_bumper);
            /*if (gamepad1.left_stick_y > 0) {
                if (armPosition > 0) {
                    armPosition += -gamepad1.left_stick_y*armSpeed;
                }
                //arm.simpleArmMove(60);
            } else if (gamepad1.left_stick_y < 0) {
                if (armPosition <= 130) {
                    armPosition += -gamepad1.left_stick_y*armSpeed;
                }
                //arm.simpleArmMove(30);
            }
            telemetry.addData("degrees:", armPosition);
            arm.simpleArmMove(armPosition, armSpeed);
            if (gamepad1.left_bumper && !Lbumper) {
                if (clawState) {
                    arm.clawClose();
                    clawState = false;
                } else {
                    arm.clawOpen();
                    clawState = true;
                }
                Lbumper = true;
            }
            if (!gamepad1.left_bumper){
                Lbumper = false;
            }

            if (gamepad1.b) {
                arm.wrist = 40;
            } else {
                arm.wrist = 23;
            }*/


        }

    }
}