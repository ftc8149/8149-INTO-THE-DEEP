package org.firstinspires.ftc.teamcode.auto.testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;
import org.firstinspires.ftc.teamcode.subsystems.LiftSystem;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;



@TeleOp(name="liftFinding", group="Testing")


public class liftFinding extends LinearOpMode {
    final double FORWARD_TICKS_PER_CM = 13;
    final double SIDEWAYS_TICKS_PER_CM = 13;
    //82.5
    double currentY = 0;
    double lastY = 0;

    double currentX = 0;
    double lastX = 0;
    double remainingY = 0;
    double remainingX = 0;

    double currentPosTicks = 0;
    double remainingPosTicks = 0;
    double lastPosTicks = 0;

    double currentNegTicks = 0;
    double remainingNegTicks = 0;
    double lastNegTicks = 0;
    //private LiftSystem lift


    //Initialize the Motors
    //L = Left
    //R = Right
    //B = Back
    //F = Front
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor lift;
    private DcMotor bootMotor = null;
    private Servo bootServo = null;
    //private OpticalDistanceSensor distanceSensor = null;
    private Servo intakePivot = null;

    OpticalDistanceSensor odsSensor;  // Hardware Device Object


    @Override
    public void runOpMode() {
        //Initialize the motors as variables. Declared as Null above
        //Make the name of the motor in the DS the same as in the program


        //lift = new LiftSystem(hardwareMap);
        lift = hardwareMap.get(DcMotor.class, "lift");
        bootServo = hardwareMap.get(Servo.class, "bootServo");
        bootMotor = hardwareMap.get(DcMotor.class, "bootMotor");
        odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "distanceSensor");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        //distanceSensor = hardwareMap.get(OpticalDistanceSensor.class, "distanceSensor");

    //Set the direction of the motor
        //Will need to be modified lending on how the motors are placed in the robot.

        waitForStart();
        while (opModeIsActive()) {
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double bootPower = (-gamepad1.left_trigger) + gamepad1.right_trigger;

            //if (bootMotor.getCurrentPosition() > 500) {
              //  bootPower = Math.max(bootPower, 0);
            //} else if (bootMotor.getCurrentPosition() < 0) {
              //  bootPower = Math.min(bootPower, 0);
            //}

            double pivotSpeed = 0;
            //if (gamepad2.left_bumper) {
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
            //}

            //moveOutServo(0);


            bootMotor.setPower(bootPower);

            if (gamepad1.a) {
                bootServo.setPosition(0.0);
            }

            if (gamepad1.b) {
                bootServo.setPosition(1);
            }




            telemetry.addData("bootMotor", bootMotor.getPower());
            telemetry.addData("liftPos", lift.getCurrentPosition());
            //telemetry.addData("distanceSensor", distanceSensor.getLightDetected());
            telemetry.addData("Raw",    odsSensor.getRawLightDetected());
            telemetry.addData("Normal", odsSensor.getLightDetected());
            telemetry.update();






        }


}
}
