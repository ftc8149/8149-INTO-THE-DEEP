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
@TeleOp(name="goofyTank", group="TeleOps")


public class goofyTank extends LinearOpMode {
    //Program for the Tele-op Program for the 2024-2025 Season

    //Start a runtime clock for the program

    //R = Right
    //B = Back
    //F = Front
    private DcMotor LMotor = null;
   private  DcMotor RMotor = null;
    @Override
    public void runOpMode() {
        //Initialize the motors as variables. Declared as Null above
        //Make the name of the motor in the DS the same as in the program
        LMotor = hardwareMap.get(DcMotor.class, "left");
        RMotor = hardwareMap.get(DcMotor.class, "right");


        //Set the direction of the motors
        //Will need to be modified later depending on how the motors are placed in the robot.

        LMotor.setDirection(DcMotor.Direction.FORWARD);
       RMotor.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();



        while (opModeIsActive()) {
            RMotor.setPower((gamepad1.left_stick_y) * .6);
            LMotor.setPower(gamepad1.right_stick_y);
        }
    }
}
