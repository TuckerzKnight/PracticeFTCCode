package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MecanumTeleop", group = "W is for Wolfpack")
public class PracticeTeleop extends LinearOpMode {//Make sure the class extends OpMode
    @Override
    public void runOpMode() throws InterruptedException {
        Base robot;
        robot = new Base();
        robot.init(hardwareMap);

        DriveHardware mecanumDrive;
        mecanumDrive = new DriveHardware();
        mecanumDrive.mecanumDriveInit(hardwareMap);

        Kinematics whereAmI;
        whereAmI = new Kinematics();
        //whereAmI.ArmKineInit(telemetry); // run all computations and check for errors


        telemetry.addLine("Robot initialized");
        telemetry.update();
        waitForStart();

        //The integral factor winds up without this
        robot.elbowPID.reset();
        robot.shoulderPID.reset();

        while(opModeIsActive()){
            robot.update(telemetry); //update sensors and telemetry
            robot.stateMachine(gamepad1, gamepad2, telemetry);//convert controls to action
            robot.motorManage(); //convert targets to motor signals

            //drive the robot
            mecanumDrive.mecanumDriveUpdateV2(gamepad1, telemetry); //handle drive-specific computation
            whereAmI.encoderUpdate(); //update drive kinematics
        }
    }
}
