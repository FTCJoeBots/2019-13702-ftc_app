package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.StrictMath.abs;

/**
 * This is NOT an opmode. This is a hardware class used to abstract the hardware config for the
 * 2018 JoeBots FTC Rover Ruckus challenge. This file has been generalized to work as a base for
 * all three JoeBots FTC teams (8513, 11855, and 13702). As the season progresses, this file may be
 * customized for each individual team in their own branch.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 *
 * motor0 (left front)
 * motor1 (right front)
 * motor2 (left rear)
 * motor3 (right rear)
 * imu - navigation features
 *
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class Utility13702 {
    /* Public OpMode members. */

    // Declare Motors
    public DcMotor liftMotor = null;
    public DcMotor armMotor = null;

    // Declare Servos
    public Servo clampServo = null;
    public Servo rotClampServo = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Private Members
    private LinearOpMode myOpMode;
    private ElapsedTime runtime = new ElapsedTime();


    static final double LIFT_THREADS_PER_INCH = 0.948;
    static final double LIFT_GEAR_REDUCTION = 1;
    static final double LIFT_COUNTS_PER_MOTOR_REV = 4.0;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_THREADS_PER_INCH * LIFT_GEAR_REDUCTION * LIFT_COUNTS_PER_MOTOR_REV);

    static final double ARM_THREADS_PER_INCH = 777;
    static final double ARM_GEAR_REDUCTION = 777;
    static final double ARM_COUNTS_PER_MOTOR_REV = 777;
    static final double ARM_COUNTS_PER_INCH = (ARM_THREADS_PER_INCH * ARM_GEAR_REDUCTION * ARM_COUNTS_PER_MOTOR_REV);


    static final double CLAMP_OPEN_POSITION = 0.99;
    static final double CLAMP_CLOSED_POSITION = 0.01;

    static final double CLAMP_HORIZONTAL_POSITION = 0.99;
    static final double CLAMP_VERTICAL_POSITION = 0.01;

    /* Constructor */
    public Utility13702() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode opMode) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        myOpMode = opMode;

        // Define and Initialize Motors
        liftMotor = hwMap.dcMotor.get("liftMotor");
        armMotor = hwMap.dcMotor.get("armMotor");

        clampServo = hwMap.servo.get("clampServo");
        rotClampServo = hwMap.servo.get("rotClampServo");

        // Set Default Motor Directions
        liftMotor.setDirection(DcMotor.Direction.FORWARD); //set to FORWARD (UP) if using AndyMark motors
        armMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        liftMotor.setPower(0);
        armMotor.setPower(0);

       // clampServo.setPosition(CLAMP_CLOSED_POSITION);
       // rotClampServo.setPosition(CLAMP_VERTICAL_POSITION);

        myOpMode.telemetry.addLine("initialized motor power to zero");
        myOpMode.telemetry.update();

        myOpMode.telemetry.addLine("initialized other motor power to zero");
        myOpMode.telemetry.update();


        // Set all drive motors to run without encoders.
        // May want to switch to  RUN_USING_ENCODERS during autonomous
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }


    /***
     * void setMode(DcMotor.RunMode mode ) Set all drive motors to same mode.
     * @param mode    Desired Motor mode.
     */
    public void setMode(DcMotor.RunMode mode) {
        liftMotor.setMode(mode);
        armMotor.setMode(mode);
    }

    public void liftMotorInches(double inches, double power){

        // Declare needed variables
        int newliftMotorTarget;


        // Check to make sure the OpMode is still active; If it isn't don't run the method
        if (myOpMode.opModeIsActive()) {

            // Determine new target positions for each wheel
            newliftMotorTarget = liftMotor.getCurrentPosition() + (int) (inches * LIFT_COUNTS_PER_INCH);

            // Send target Positions to motors
            liftMotor.setTargetPosition(newliftMotorTarget);

            // Set Robot to RUN_TO_POSITION mode
            setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the runtime
            runtime.reset();

            // Set the motors back to standard mode
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public void armMotorInches(double inches, double power){

        // Declare needed variables
        int newArmMotorTarget;


        // Check to make sure the OpMode is still active; If it isn't don't run the method
        if (myOpMode.opModeIsActive()) {

            // Determine new target positions for each wheel
            newArmMotorTarget = armMotor.getCurrentPosition() + (int) (inches * ARM_COUNTS_PER_INCH);

            // Send target Positions to motors
            armMotor.setTargetPosition(newArmMotorTarget);

            // Set Robot to RUN_TO_POSITION mode
            setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the runtime
            runtime.reset();

            // Set the motors back to standard mode
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

    public void openClamp(){

        myOpMode.telemetry.addData("opening clamp", clampServo.getPosition());
        myOpMode.telemetry.update();
        clampServo.setPosition(CLAMP_OPEN_POSITION);

    }

    public void closeClamp(){

        myOpMode.telemetry.addData("closing clamp", clampServo.getPosition());
        myOpMode.telemetry.update();
        clampServo.setPosition(CLAMP_CLOSED_POSITION);

    }

    public void toggleClampOpen(){

       double clampServoPosition = clampServo.getPosition();

        if(clampServoPosition < .5){

            myOpMode.telemetry.addLine("servo is closed, opening");
            myOpMode.telemetry.update();
            openClamp();

        }else{

            myOpMode.telemetry.addLine("servo is opened, closing");
            myOpMode.telemetry.update();
            closeClamp();
        }

    }


    public void clampHorizontal(){

        myOpMode.telemetry.addData("moving clamp horizontal", rotClampServo.getPosition());
        myOpMode.telemetry.update();
        rotClampServo.setPosition(CLAMP_HORIZONTAL_POSITION);

    }

    public void clampVertical(){

        myOpMode.telemetry.addData("moving clamp vertical", rotClampServo.getPosition());
        myOpMode.telemetry.update();
        rotClampServo.setPosition(CLAMP_VERTICAL_POSITION);

    }

    public void toggleClampDirection(){

        if(rotClampServo.getPosition() < 0.5){

            myOpMode.telemetry.addLine("clamp is vertical, moving horizontal");
            myOpMode.telemetry.update();
            clampHorizontal();

        }else{

            myOpMode.telemetry.addLine("clamp is horizontal, moving vertical");
            myOpMode.telemetry.update();
            clampVertical();

        }

    }
}