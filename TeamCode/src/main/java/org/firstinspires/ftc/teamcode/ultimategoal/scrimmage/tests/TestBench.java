package org.firstinspires.ftc.teamcode.ultimategoal.scrimmage.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TestBench")
public class TestBench extends LinearOpMode {

    //Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    private DcMotorSimple simpleMotor1 = null;
    private DcMotorSimple simpleMotor2 = null;
    private Servo Servo;
    private CRServo CRservo = null;
    DigitalChannel limitSwitch;


    private boolean is1YPressed = false;
    private boolean slowDrive = false;
    private double servoPosition = 0.16;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        simpleMotor1 = hardwareMap.get(DcMotorSimple.class, "simpleMotor1");
        simpleMotor2 = hardwareMap.get(DcMotorSimple.class, "simpleMotor2");
        Servo = hardwareMap.servo.get("Servo");
        CRservo = hardwareMap.get(CRServo.class, "CRservo");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");

        Servo.setPosition(servoPosition);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double mp1;
            double mp2;
            double mp3;
            double mp4;
            double smp1 = gamepad1.left_stick_x;
            double smp2 = gamepad1.right_stick_x;

            if (gamepad1.y) {
                if (!is1YPressed) {
                    is1YPressed = true;
                    slowDrive = !slowDrive;
                } else {
                    is1YPressed = false;
                }
            }

            if (gamepad1.left_trigger > 0) {                                                        //Capstone Servo Control
                servoPosition = gamepad1.left_trigger;
                servoPosition = Range.clip(servoPosition, .16, .75);
            } else {
                servoPosition = 0.16;
            }
            Servo.setPosition(servoPosition);
            telemetry.addData("Servo pos: %s", Servo.getPosition());
            telemetry.update();

            if (gamepad2.left_bumper) {
                simpleMotor2.setPower(smp1);
            }
            else if (gamepad2.right_bumper) {
                simpleMotor2.setPower(smp1);
            }
            else {
                simpleMotor2.setPower(0);

            }


        }
    }
}
