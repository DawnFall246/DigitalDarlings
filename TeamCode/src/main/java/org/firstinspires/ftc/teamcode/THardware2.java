package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Front right drive motor:  "front_right_drive"
 * Motor channel:  Back right drive motor:   "back_right_drive"
 * Motor channel:  Front left drive motor:   "front_left_drive"
 * Motor channel:  Back left drive motor:    "back_left_drive"
 */
public class THardware2 implements ArmHardware
{
    /* Public OpMode members. */
    public DcMotor MFR   = null;
    public DcMotor MFL   = null;
    public DcMotor MBR   = null;
    public DcMotor MBL   = null;

    public DcMotorEx ArmBase  = null;
    public DcMotorEx ArmJoint = null;

    public static double p = 10.6;
    public static double i = 1.4;
    public static double d = 0;
    public static double f = 0;

    public Servo EndJoint = null;
    public Servo Gripper  = null;
    public Servo FoundationMover = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public THardware2(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        MFR   = hwMap.dcMotor.get("fr");
        MFL   = hwMap.dcMotor.get("fl");
        MBR   = hwMap.dcMotor.get("br");
        MBL   = hwMap.dcMotor.get("bl");

        ArmBase  = (DcMotorEx) hwMap.dcMotor.get("base");
        ArmJoint = (DcMotorEx) hwMap.dcMotor.get("joint");

        EndJoint = hwMap.servo.get("wrist");
        Gripper  = hwMap.servo.get("gripper");
        FoundationMover = hwMap.servo.get("foundation");


        MFR.setDirection(DcMotor.Direction.FORWARD);
        MFL.setDirection(DcMotor.Direction.REVERSE);
        MBR.setDirection(DcMotor.Direction.FORWARD);
        MBL.setDirection(DcMotor.Direction.REVERSE);

        ArmBase.setDirection(DcMotor.Direction.FORWARD);
        ArmJoint.setDirection(DcMotor.Direction.FORWARD);


        MFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ArmBase.setVelocityPIDFCoefficients(p, i, d, f);
        ArmBase.setPositionPIDFCoefficients(p);
        ArmJoint.setVelocityPIDFCoefficients(p, i, d, f);
        ArmJoint.setPositionPIDFCoefficients(p);

        // Set all motors to zero power
        MFR.setPower(0);
        MFL.setPower(0);
        MBR.setPower(0);
        MBL.setPower(0);

        EndJoint.setPosition(0.1);
        Gripper.setPosition(0.5);

    }

    /**
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
