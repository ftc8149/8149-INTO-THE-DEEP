package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Name of the program is a tradition. Last year the program was Yes8.
//This year the Tele-Op programs will follow the same format with the prefix
//of YesMk2. :D -LF
@TeleOp(name="tank", group="TeleOps")


public class TankWithTracks extends LinearOpMode {
    //Program for the Tele-op Program for the 2024-2025 Season

    //Start a runtime clock for the program

    //R = Right
    //B = Back
    //F = Front
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor FLMotor = null;
    private  DcMotor FRMotor = null;
    @Override
    public void runOpMode() {
        //Initialize the motors as variables. Declared as Null above
        //Make the name of the motor in the DS the same as in the program
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");

        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");


        //Set the direction of the motors
        //Will need to be modified later depending on how the motors are placed in the robot.

        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();



        while (opModeIsActive()) {
            BRMotor.setPower((gamepad1.left_stick_y)+gamepad1.right_stick_x);
            BLMotor.setPower((gamepad1.left_stick_y)-gamepad1.right_stick_x);

            FLMotor.setPower(gamepad1.left_stick_y-gamepad1.right_stick_x);
            FRMotor.setPower(gamepad1.left_stick_y+gamepad1.right_stick_x);
        }
    }
}
