package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "GotoPos", group = "Undertow")
public class GoToPositionOpmode extends LinearOpMode {//Make sure the class extends OpMode

    DcMotorEx pitch1, pitch2, ext1, ext2;
    int pitch1Pos, pitch2Pos, ext1Pos, ext2Pos;
    int index = 0;
    boolean bDebounce = false;
    boolean aDebounce = false;
    double globalPower = 0.2;
    public enum State {
        IDLE,
        DEPOSIT,
        INTAKE,
        BACKPUT
    }
    public static State state = State.IDLE;
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

        pitch1 = hardwareMap.get(DcMotorEx.class, "rotation1");
        pitch2 = hardwareMap.get(DcMotorEx.class, "rotation2");
        ext1 = hardwareMap.get(DcMotorEx.class, "extension1");
        ext2 = hardwareMap.get(DcMotorEx.class, "extension2");
        pitch1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitch2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ext1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ext2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pitch1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pitch2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ext1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ext2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pitch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(opModeIsActive()){
            robot.update(telemetry); //update sensors and telemetry
            if (gamepad1.b) {
                if (index < 3) {
                    index++;
                } else {
                    index = 0;
                }
                bDebounce = true;
            } else {
                bDebounce = false;
            }

            switch (index) {
                case 0:
                    pitch1.setPower(globalPower*gamepad1.left_stick_y);
                case 1:
                    pitch2.setPower(globalPower*gamepad1.left_stick_y);
                case 2:
                    ext1.setPower(globalPower*gamepad1.left_stick_y);
                case 3:
                    ext2.setPower(globalPower*gamepad1.left_stick_y);
            }

            /*switch (state) {
                case State.IDLE:
                    if (gamepad1.a){
                        aDebounce = true;
                        state = State.INTAKE;
                    }
            }*/
            telemetry.addData("pitch1: ", pitch1.getCurrentPosition());
            telemetry.addData("pitch2: ", pitch2.getCurrentPosition());
            telemetry.addData("ext1: ", ext1.getCurrentPosition());
            telemetry.addData("ext2: ", ext2.getCurrentPosition());
            //drive the robot
            mecanumDrive.mecanumDriveUpdateV2(gamepad1, telemetry); //handle drive-specific computation
            whereAmI.encoderUpdate(); //update drive kinematics
        }
    }
}
