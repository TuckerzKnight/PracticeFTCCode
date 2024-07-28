package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleII", group = "Anti-non-Gm0")
public class TeleopV2 extends LinearOpMode {//Make sure the class extends OpMode
    @Override
    public void runOpMode() throws InterruptedException {

        Drivebase mecanumDrive;
        mecanumDrive = new Drivebase();
        mecanumDrive.init(hardwareMap);

        Main robot = new Main();
        robot.init(hardwareMap);


        telemetry.addLine("Robot initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            //drive the robot
            mecanumDrive.update(gamepad1, hardwareMap); //handle drive-specific computation
            robot.update(telemetry);
        }
    }
}
