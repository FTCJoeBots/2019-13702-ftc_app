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
    public DcMotor leftIntake = null;
    public DcMotor rightIntake = null;

    // Declare Servos
    public Servo clampServo = null;
    public Servo rotClampServo = null;
    public Servo leftIntakeServo = null;
    public Servo rightIntakeServo = null;
    public Servo grabberServo = null;

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


    static final double CLAMP_OPEN_POSITION = 0.3;
    static final double CLAMP_CLOSED_POSITION = 0.15;

    static final double CLAMP_HORIZONTAL_POSITION = 0.4;
    static final double CLAMP_VERTICAL_POSITION = 0.05;
    static final double CLAMP_VERTICAL_SLANTED_POSITION = 0.1;
    static final double CLAMP_HORIZOTAL_SLANTED_POSITION = 0.45;

    static final double GRABBER_CLOSED_POSITION = 0.85;
    static final double GRABBER_OPEN_POSITION = 0.05;

    static final double RIGHT_INTAKE_SERVO_UP_POSITION = 0.25;
    static final double RIGHT_INTAKE_SERVO_DOWN_POSITION = 0.01;

    static final double LEFT_INTAKE_SERVO_OUT_POSITION = 0.4;
    static final double LEFT_INTAKE_SERVO_IN_POSITION = 0.9;

    double leftIntakeServoCurr = LEFT_INTAKE_SERVO_IN_POSITION;

    boolean rightIntakeServoUp = true;
    boolean isClampVertical = true;

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
        leftIntake = hwMap.dcMotor.get("leftIntake");
        rightIntake = hwMap.dcMotor.get("rightIntake");

        clampServo = hwMap.servo.get("clampServo");
        rotClampServo = hwMap.servo.get("rotClampServo");
        leftIntakeServo = hwMap.servo.get("leftIntakeServo");
        rightIntakeServo = hwMap.servo.get("rightIntakeServo");
        grabberServo = hwMap.servo.get("grabberServo");

        // Set Default Motor Directions
        liftMotor.setDirection(DcMotor.Direction.FORWARD); //set to FORWARD (UP) if using AndyMark motors
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        liftMotor.setPower(0);
        armMotor.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        clampServo.setPosition(CLAMP_CLOSED_POSITION);
        rotClampServo.setPosition(CLAMP_VERTICAL_POSITION);
        leftIntakeServo.setPosition(LEFT_INTAKE_SERVO_IN_POSITION);
        rightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_UP_POSITION);
        grabberServo.setPosition(GRABBER_OPEN_POSITION);

        myOpMode.telemetry.addLine("initialized motor power to zero");
        myOpMode.telemetry.update();

        myOpMode.telemetry.addLine("initialized other motor power to zero");
        myOpMode.telemetry.update();


        // Set all drive motors to run without encoders.
        // May want to switch to  RUN_USING_ENCODERS during autonomous
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        leftIntake.setMode(mode);
        rightIntake.setMode(mode);
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

    public void moveLift(double power){
        liftMotor.setPower(power);
    }


    public void armMotorInches(double inches, double power) {

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


    public void moveArm(double power){
            armMotor.setPower(power);
    }

/////////////////////////////////////////////////////////////////////////////
    public void openClampPos(){

        clampServo.setPosition(CLAMP_OPEN_POSITION);

    }

    public void closeClampPos(){

        clampServo.setPosition(CLAMP_CLOSED_POSITION);

    }

/////////////////////////////////////////////////////////////////////////////
    public void clampHorizontalPos(){

        rotClampServo.setPosition(CLAMP_HORIZONTAL_POSITION);

    }

    public void clampVertical(){

        rotClampServo.setPosition(CLAMP_VERTICAL_POSITION);

    }

    public void clampVerticalSlanted(){

        rotClampServo.setPosition(CLAMP_VERTICAL_SLANTED_POSITION);

    }

    public void clampHorizotalSlanted(){

        rotClampServo.setPosition(CLAMP_HORIZOTAL_SLANTED_POSITION);

    }

    public void clampDirectionToggle(){



    }
/////////////////////////////////////////////////////////////////////////////

    //the three teleOp methods for clamp control

    public void grabBlock(){

        clampVerticalSlanted();
        openClampPos();

        myOpMode.telemetry.addLine("grab block position");
        myOpMode.telemetry.update();

        }

    public void closeClamp(){

        clampVertical();
        closeClampPos();

        myOpMode.telemetry.addLine("close block position");
        myOpMode.telemetry.update();

    }

    public void clampClosedHorizontal(){

       clampHorizontalPos();
       closeClampPos();

        myOpMode.telemetry.addLine("close horizontal position");
        myOpMode.telemetry.update();

    }

    public void clampOpenHorizontal(){

        clampHorizotalSlanted();
        openClampPos();

        myOpMode.telemetry.addLine("open horizontal position");
        myOpMode.telemetry.update();

    }

///////////////////////////////////////////////////////////////////////////////////////////////////
    public void spinIntake(){

        myOpMode.telemetry.addLine("start intake wheels");
        myOpMode.telemetry.update();

        //spin both intake motors
        leftIntake.setPower(0.95);
        rightIntake.setPower(0.95);
    }

    public void stopIntake(){

        myOpMode.telemetry.addLine("stop intake wheels");
        myOpMode.telemetry.update();

        //stops both intake motors
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }

//////////////////////////////////////////////////////////////////////////////////////////////////

    public void closeGrabber(){
        grabberServo.setPosition(GRABBER_CLOSED_POSITION);

        myOpMode.telemetry.addLine("foudation grabber servo closing");
        myOpMode.telemetry.update();
    }

    public void openGrabber(){
        grabberServo.setPosition(GRABBER_OPEN_POSITION);

        myOpMode.telemetry.addLine("foudation grabber servo opening");
        myOpMode.telemetry.update();
    }
///////////////////////////////////////////////////////////////////////////////////////////////////

    public void rightIntakeServoUpPos(){
        rightIntakeServoUp = true;

        rightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_DOWN_POSITION);

        rightIntakeServoUp = false;
    }

    public void rightIntakeServoDownPos(){
        rightIntakeServoUp = false;

        rightIntakeServo.setPosition(RIGHT_INTAKE_SERVO_UP_POSITION);

        rightIntakeServoUp = true;
    }

    public void toggleRightIntakeServo(){

        if(rightIntakeServoUp){
            rightIntakeServoUpPos();
        }else{
            rightIntakeServoDownPos();
        }
    }
///////////////////////////////////////////////////////////////////////////////////////////////////

    public void leftIntakeServoIn(){
        leftIntakeServo.setPosition(LEFT_INTAKE_SERVO_IN_POSITION);
    }

    public void leftIntakeServoOut(){
        leftIntakeServo.setPosition(LEFT_INTAKE_SERVO_OUT_POSITION);
    }

    public void moveLeftIntakeServo(boolean direction){
        //true = left, false = right
        if(leftIntakeServoCurr < LEFT_INTAKE_SERVO_IN_POSITION || leftIntakeServoCurr > LEFT_INTAKE_SERVO_OUT_POSITION){
            if(direction) {

                leftIntakeServoCurr = leftIntakeServoCurr - 0.1;
            }else{

                leftIntakeServoCurr = leftIntakeServoCurr + 0.1;
            }

                leftIntakeServo.setPosition(leftIntakeServoCurr);

        }



    }
}