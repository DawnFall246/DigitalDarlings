package org.firstinspires.ftc.teamcode;

/**
 * Created by Miranda on 10/20/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * T
 */

@TeleOp(name="Adjust Arm", group="TeleOp")
//@Disabled
public class ArmAdjust extends LinearOpMode {

    /* Declare OpMode members. */
    THardware1 robot           = new THardware1();   // Use a hardware
    ElapsedTime runtime = new ElapsedTime();

    //Double for more precision
    @Override
    public void runOpMode() throws InterruptedException {


        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        ArtArm reach = new ArtArm(robot, 15, 15, 3, 1, 1, 2);

        int base = 0;
        int joint = 0;

        double x = 0;
        double y = 0;
        int[] pos = {0, 0, 0};

        double wrist = robot.EndJoint.getPosition();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ////////////////////////////////////////////ARTICULATED ARM/////////////////////////////////////////////////////////
            /*
            base += (int) gamepad1.right_stick_y * 10;
            joint += (int) gamepad1.right_stick_x * 10;
            wrist -= gamepad1.left_stick_y * 0.01;

            if (wrist >= 1)
                wrist = 1;
            else if (wrist <= 0)
                wrist = 0;

            robot.ArmBase.setTargetPosition(base);
            robot.ArmBase.setPower(0.4);
            robot.ArmJoint.setTargetPosition(joint);
            robot.ArmJoint.setPower(0.4);

            robot.EndJoint.setPosition(wrist);
            */

            if(x > reach.maxX())
                x = reach.maxX();
            if(x < 0)
                x = 0;
            else
                x += gamepad1.left_stick_x;


            if(y > reach.maxY())
                y = reach.maxY();
            if(y < 0)
                y = 0;
            else
                y -= gamepad1.left_stick_y;


            pos = reach.goToXY(x, y);
            robot.ArmBase.setTargetPosition(pos[0]);
            robot.ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmBase.setPower(0.3);
            robot.ArmJoint.setTargetPosition(pos[1]);
            robot.ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmJoint.setPower(0.3);
            robot.EndJoint.setPosition(pos[2]/360.0);

            if (gamepad1.left_bumper)
                robot.FoundationMover.setPosition(.2);
            else if(gamepad1.right_bumper)
                robot.FoundationMover.setPosition(.9);

            telemetry.addData("X: ", reach.getX());
            telemetry.addData("Y: ", reach.getY());
            telemetry.addData(" " + pos[0] + " " + pos[1] + " " + pos[2], "");
            telemetry.addData("Base: ", robot.ArmBase.getCurrentPosition());
            telemetry.addData("Joint: ", robot.ArmJoint.getCurrentPosition());
            telemetry.addData("Servo: ", robot.EndJoint.getPosition());
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
