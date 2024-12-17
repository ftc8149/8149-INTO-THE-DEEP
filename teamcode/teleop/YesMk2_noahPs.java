package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Name of the program is a tradition. Last year the program was Yes8.
//This year the Tele-Op programs will follow the same format with the prefix
//of YesMk2. :D -LF
@TeleOp(name="YesMk2Noah", group="TeleOps")

@Disabled
public class YesMk2_noahPs extends LinearOpMode {
    //Program for the Tele-op Program for the 2024-2025 Season

    //Start a runtime clock for the program
    private ElapsedTime runtime = new ElapsedTime();
    private double minutes = 0;
    private double seconds = 0;

    //Initialize the Motors
    //L = Left
    //R = Right
    //B = Back
    //F = Front
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor lift = null;
    private DcMotor winch = null;
    private Servo outtake = null;
    private Servo intakeServo = null;
    private DcMotor intakeMotor = null;
    private Servo intakePivot = null;
    private Servo clippyServo = null;

    @Override
    public void runOpMode() {
        //Initialize the motors as variables. Declared as Null above
        //Make the name of the motor in the DS the same as in the program
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");

        lift = hardwareMap.get(DcMotor.class, "lift");

        outtake = hardwareMap.get(Servo.class, "outtake");
        outtake.scaleRange(0, 1);


        intakePivot = hardwareMap.get(Servo.class, "intakePivot");


        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        clippyServo = hardwareMap.get(Servo.class, "clippyServo");
        clippyServo.scaleRange(0, 1);

        winch = hardwareMap.get(DcMotor.class, "Winch");

        double speedFactor = 1;


        //Set the direction of the motors
        //Will need to be modified later depending on how the motors are placed in the robot.

        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        winch.setDirection(DcMotor.Direction.FORWARD);

        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Print statement to the DS
        //Name of data followed by data value
        telemetry.addData("Status", "Init");
        //update the DS screen
        telemetry.update();

        //Wait for the Program to start for it to continue
        //Init will be pressed at this point, but the play button will not be pressed
        waitForStart();

        //Reset the Runtime clock so that the runtime of the game is seen on the Tele-Op screen
        runtime.reset();

        while (opModeIsActive()) {
            //This drive code uses a drive system where the left joystick controls Forward/Backward Left/Right
            //And the right joystick rotates the robot on it's axis

            double zAxisInput = -gamepad1.right_stick_x; //Rotation on the axis input
            double xAxisInput = -gamepad1.left_stick_x; //Strafing axis
            double yAxisInput = gamepad1.left_stick_y; //Forward/Reverse axis

            //Combine joystick inputs for the power to the wheels
            //Google a diagram of how the math works
            //Too long to write here, but not crazy complicated :D

            double FLPower = (yAxisInput + xAxisInput + zAxisInput) * speedFactor;
            double FRPower = (yAxisInput - xAxisInput - zAxisInput) * speedFactor;
            double BLPower = (yAxisInput - xAxisInput + zAxisInput) * speedFactor;
            double BRPower = (yAxisInput + xAxisInput - zAxisInput) * speedFactor;

            //Ensure that the power value does not exceed 1 for the motors
            //Don't want to blow up a motor; good idea luke -EG

            //Declare a max variable for the math. Will be set to 1 during the program
            double max;
            max = Math.max(Math.abs(FLPower), Math.abs(FRPower));
            max = Math.max(max, Math.abs(BLPower));
            max = Math.max(max, Math.abs(BRPower));

            //Divide all the powers by the max so that the max power is 1
            if (max > 1.0) {
                FLPower /= max;
                FRPower /= max;
                BLPower /= max;
                BRPower /= max;
            }

            //Send the power to the motors with the variables defined above
            FLMotor.setPower(FLPower);
            FRMotor.setPower(FRPower);
            BLMotor.setPower(BLPower);
            BRMotor.setPower(BRPower);

            //lift stuff
            /**
             if (gamepad2.right_stick_x) {
             lift.setPower(-1);
             lift.setPower(0);
             }
             if (gamepad2.right_stick_x >= 0.5) {
             lift.setPower(1);
             lift.setPower(0);
             }
             **/

            lift.setPower((-gamepad1.left_trigger) + (gamepad1.right_trigger));


            if (gamepad1.a) {
                outtake.setPosition(0.0);
            }
            if (gamepad1.b) {
                outtake.setPosition(0.6);
            }

            if (gamepad1.right_bumper) {
                speedFactor = .5;
            }
            if (gamepad1.left_bumper) {
                speedFactor = 1;
            }

            double intakeSpeed = 0;


            if (gamepad2.x) {
                intakeSpeed = 0;
                intakeServo.setPosition(intakeSpeed);
            } else if (gamepad2.y) {
                intakeSpeed = 1;
                intakeServo.setPosition(intakeSpeed);
            } else {
                intakeSpeed = 0.46;
                intakeServo.setPosition(intakeSpeed);
            }

            intakeMotor.setPower((-gamepad2.left_trigger) + gamepad2.right_trigger);

            double pivotSpeed = 0;
            if (gamepad2.left_bumper) {
                if (gamepad2.a) {
                    pivotSpeed = 0;
                    intakePivot.setPosition(pivotSpeed);
                } else if (gamepad2.b) {
                    pivotSpeed = 1;
                    intakePivot.setPosition(pivotSpeed);
                } else {
                    pivotSpeed = .5;
                    intakePivot.setPosition(pivotSpeed);
                }
            }
            if (gamepad1.dpad_down) {
                clippyServo.setPosition(1);
            }
            if (gamepad1.dpad_right) {
                clippyServo.setPosition(0);
            }
            if (gamepad1.dpad_left) {
                winch.setPower(-1);
            } else if (gamepad1.dpad_up) {
                winch.setPower(1);
            } else {
                winch.setPower(0);
            }


            //Show the elapsed time of the program and show the power of the motors
            telemetry.addData("Status", "Runtime" + runtime.toString());
            telemetry.addData("Front Left/Right", "%4.2f, %4.2f", FLPower, FRPower);
            telemetry.addData("Back Left/Right", "%4.2f, %4.2f", BLPower, BRPower);
            telemetry.addData("servoPos", outtake.getPosition());
            telemetry.addData("intakePosNum", intakeSpeed);
            telemetry.addData("intakeRealPos", intakeServo.getPosition());
            telemetry.addData("lift Power", lift.getPower());
            telemetry.addData("liftPos", lift.getCurrentPosition());
            telemetry.update();
            //updates the screen on the DS -> Driver station
            //RC ->Robot controller

        }
    }
}