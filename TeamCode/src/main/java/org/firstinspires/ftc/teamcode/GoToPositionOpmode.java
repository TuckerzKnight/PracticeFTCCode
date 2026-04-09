package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "GotoPos", group = "Undertow")
public class GoToPositionOpmode extends LinearOpMode {//Make sure the class extends OpMode

    DcMotorEx pitch1, pitch2, ext1, ext2;
    int pitch1Pos, pitch2Pos, ext1Pos, ext2Pos;
    Servo claw, wristY, wristX;
    public static double clawPos, wristYPos, wristXPos;
    int index = 0;
    boolean bDebounce = false;
    boolean aDebounce = false;
    boolean xDebounce = false;
    boolean yDebounce = false;
    double globalPower = 0.2;
    public static int pivotTarget = 20;
    public static int extendoTarget = 0;
    double pivotTPD = (28*3.61*3.61*3.61*54/24)/360;
    double extendoTPMM = 6.2536;
    public enum State {
        IDLE,
        DEPOSIT,
        INTAKE,
        BACKPUT,
        BONUS;
    }
    public static State state = State.IDLE;
    ElapsedTime stateTimer = new ElapsedTime();
    int servoMoveTime = 200; //in milliseconds
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
        pitch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitch2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitch1.setDirection(DcMotorSimple.Direction.REVERSE);
        pitch2.setDirection(DcMotorSimple.Direction.REVERSE);
        ext1.setDirection(DcMotorSimple.Direction.FORWARD);
        ext2.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.get(Servo.class, "Claw");
        wristX = hardwareMap.get(Servo.class, "WristX");
        wristY = hardwareMap.get(Servo.class, "WristY");
        claw.setDirection(Servo.Direction.FORWARD);
        wristX.setDirection(Servo.Direction.FORWARD);
        wristY.setDirection(Servo.Direction.FORWARD);

        pitch1.setTargetPosition(pivotTarget);
        pitch2.setTargetPosition(pivotTarget);
        ext1.setTargetPosition(extendoTarget);
        ext2.setTargetPosition(extendoTarget);
        pitch1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pitch2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ext1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ext2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            switch (state) {
                case IDLE:
                    //tune these set positions
                    pivotTarget = (int)(15*pivotTPD);
                    extendoTarget = 0;
                    clawPos = 0;
                    wristXPos = 1.0;
                    wristYPos = 0.5;

                    if (gamepad1.a){
                        aDebounce = true;
                        state = State.INTAKE;
                    } else {
                        aDebounce = false;
                    }
                    break;
                case INTAKE:
                    pivotTarget = (int)(15*pivotTPD);
                    extendoTarget = (int)(500*extendoTPMM);
                    clawPos = 0;
                    wristXPos = 1;
                    wristYPos = 0.5;
                    if (!aDebounce && gamepad1.a && stateTimer.milliseconds() > servoMoveTime*2) {
                        clawPos = 0.5;
                        stateTimer.reset();
                    } else if (!gamepad1.a) {
                        aDebounce = false;
                    }
                    if (stateTimer.milliseconds() > servoMoveTime) {
                        wristYPos = 0.75;
                    } else if (stateTimer.milliseconds() > servoMoveTime*2) {
                        state = State.IDLE;
                    }
                    break;
                case BONUS:
                    pivotTarget = (int)(40*pivotTPD);
                    extendoTarget = 0;
                    clawPos = 0;
                    wristXPos = 1;
                    wristYPos = 1;
                    if (!xDebounce && gamepad1.x) {
                        clawPos = 0.5;
                        stateTimer.reset();
                    } else if (!gamepad1.x) {
                        aDebounce = false;
                    }
                    break;
                case DEPOSIT:
                    pivotTarget = (int)(90*pivotTPD);
                    extendoTarget = (int)(530*extendoTPMM);
                    clawPos = 0;
                    wristXPos = 1;
                    wristYPos = 0.5;
                    if (!aDebounce && gamepad1.y && stateTimer.milliseconds() > servoMoveTime*2) {
                        clawPos = 1;
                        stateTimer.reset();
                    } else if (!gamepad1.y) {
                        yDebounce = false;
                    }
                    if (stateTimer.milliseconds() > servoMoveTime) {
                        wristYPos = 0.75;
                    } else if (stateTimer.milliseconds() > servoMoveTime*2) {
                        state = State.IDLE;
                    }
                    break;
                default:
                    state = State.IDLE;
            }
            pitch1.setTargetPosition(pivotTarget);
            pitch2.setTargetPosition(pivotTarget);
            ext1.setTargetPosition(extendoTarget);
            ext2.setTargetPosition(extendoTarget);
            claw.setPosition(clawPos);
            wristX.setPosition(wristXPos);
            wristY.setPosition(wristYPos);
            telemetry.addData("pitch1: ", pitch1Pos);
            telemetry.addData("pitch2: ", pitch2Pos);
            telemetry.addData("ext1: ", ext1Pos);
            telemetry.addData("ext2: ", ext2Pos);
            //drive the robot
            mecanumDrive.mecanumDriveUpdateV2(gamepad1, telemetry); //handle drive-specific computation
            whereAmI.encoderUpdate(); //update drive kinematics
        }
    }
}
