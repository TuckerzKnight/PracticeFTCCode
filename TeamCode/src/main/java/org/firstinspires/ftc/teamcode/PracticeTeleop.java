package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MecanumTeleop")
public class PracticeTeleop extends LinearOpMode {//Make sure the class extends OpMode
    @Override
    public void runOpMode() throws InterruptedException {
        Base robot;
        robot = new Base();
        robot.init(hardwareMap);

        DriveHardware mecanumDrive;
        mecanumDrive = new DriveHardware();
        mecanumDrive.mecanumDriveInit(hardwareMap);

        Localization whereAmI;
        whereAmI = new Localization();
        whereAmI.locaInit(hardwareMap);


        telemetry.addLine("Robot initialized");
        telemetry.update();
        waitForStart();


        while(opModeIsActive()){
            robot.update(telemetry);

            //drive the robot
            mecanumDrive.mecanumDriveUpdateV2(gamepad1, telemetry);
            whereAmI.encoderUpdate(telemetry);
            telemetry.addData("EncoderHeading", whereAmI.heading);
            telemetry.addData("EncoderX", whereAmI.X);
            telemetry.addData("EncoderY", whereAmI.Y);
            telemetry.update();
        }
    }
}
