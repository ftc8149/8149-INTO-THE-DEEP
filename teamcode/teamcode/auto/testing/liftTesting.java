package org.firstinspires.ftc.teamcode.auto.testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.LiftSystem;


@Autonomous(name="liftTesting", group="Testing")


public class liftTesting extends LinearOpMode {
    final double FORWARD_TICKS_PER_CM = 13;
    final double SIDEWAYS_TICKS_PER_CM = 13;
    //82.5
    double currentY = 0;
    double lastY = 0;

    double currentX = 0;
    double lastX = 0;
    double remainingY = 0;
    double remaningX = 0;

    double currentPosTicks = 0;
    double remainingPosTicks = 0;
    double lastPosTicks = 0;

    double currentNegTicks = 0;
    double remainingNegTicks = 0;
    double lastNegTicks = 0;
    private LiftSystem lift;
    private DcMotor liftMotor;





    //Initialize the Motors
    //L = Left
    //R = Right
    //B = Back
    //F = Front
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;

    @Override
    public void runOpMode() {
        liftMotor = hardwareMap.dcMotor.get("lift");

        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();



        liftMotor.setTargetPosition(1800);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
        while (liftMotor.isBusy()) {
            telemetry.addData("sdf", liftMotor.getCurrentPosition());
            telemetry.update();
        }

        sleep(1500);
        liftMotor.setTargetPosition(500);
        liftMotor.setPower(1);

        while (liftMotor.isBusy()) {
            telemetry.addData("sdf", liftMotor.getCurrentPosition());
            telemetry.update();
        }
        sleep(1500);


    }

}