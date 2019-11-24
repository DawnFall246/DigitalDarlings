package org.firstinspires.ftc.teamcode;

/**
 * Created by Miranda on 10/20/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * T
 */

@TeleOp(name="Adjust Arm", group="TeleOp")
//@Disabled
public class ArmAdjust extends LinearOpMode {

    /* Declare OpMode members. */
    THardware1 robot           = new THardware1();   // Use a hardware
    ArtArm reach = new ArtArm(robot, 15, 16.5, 3, 1, 1, 2);
    ElapsedTime runtime = new ElapsedTime();

    //Double for more precision
    @Override
    public void runOpMode() throws InterruptedException {


        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        int base = 0;
        int joint = 0;
        double wrist = robot.EndJoint.getPosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ////////////////////////////////////////////ARTICULATED ARM/////////////////////////////////////////////////////////
            base  += (int) gamepad1.right_stick_y*10;
            joint += (int) gamepad1.right_stick_x*10;
            wrist -= gamepad1.left_stick_y*0.01;

            if(wrist >= 1)
                wrist = 1;
            else if(wrist <= 0)
                wrist = 0;

            robot.ArmBase.setTargetPosition(base);
            robot.ArmBase.setPower(0.4);
            robot.ArmJoint.setTargetPosition(joint);
            robot.ArmJoint.setPower(0.4);

            robot.EndJoint.setPosition(wrist);

            telemetry.addData("Base: ", robot.ArmBase.getCurrentPosition());
            telemetry.addData("",base);
            telemetry.addData("Joint: ", robot.ArmJoint.getCurrentPosition());
            telemetry.addData("", joint);
            telemetry.addData("Servo: ", robot.EndJoint.getPosition());
            telemetry.addData("", wrist);
            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robot.waitForTick(40);
        }
    }

    public void armMove(int base, int joint){
        robot.ArmBase.setTargetPosition(base+20);
        robot.ArmBase.setPower(0.1);
        robot.ArmJoint.setTargetPosition(joint-70);
        robot.ArmJoint.setPower(0.2);
        //robot.EndJoint.setPosition(reach.getEEDeg());
    }
}
