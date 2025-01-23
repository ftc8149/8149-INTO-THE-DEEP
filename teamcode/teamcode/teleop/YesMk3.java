package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LiftSystem;

@TeleOp(name="YesMk3", group="TeleOps")
public class YesMk3 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double minutes = 0;
    private double seconds = 0;

    private LiftSystem lift;
    private Drivetrain drivetrain;
    private DcMotor bootMotor = null;
    private Servo bootServo = null;

    public void runOpMode() {
        lift = new LiftSystem(hardwareMap);
        lift.init();

        drivetrain = new Drivetrain(hardwareMap);
        drivetrain.init();

        waitForStart();

        while (opModeIsActive()) {
            drivetrain.teleopMove();

            bootMotor.setPower((-gamepad2.left_trigger) + gamepad2.right_trigger);

            if (gamepad2.dpad_up) {
                bootServo.setPosition(0.0);
            }
            if (gamepad2.dpad_down) {
                bootServo.setPosition(1.0);


            }


}}}
