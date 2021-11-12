/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;






/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="10448 teleop controlled period", group="Pushbot")
//@Disabled
public class PushbotTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot           = new HardwarePushbot();   // Use a Pushbot's hardware
    double          clawOffset      = 0;                       // Servo mid A
    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
    //hi
     static double MAX_POS     =  1.0;   
     static double MIN_POS     =  0.0;   

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double turns;
        double leftArm;
        int rotate;
        int rotate0;
        int rotate2;
        int trim;
        Servo servo;
        final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
        final int CYCLE_MS    =   50;     // period of each cycle/* Initialize the hardware variables.
         // Maximum rotational position * The init() method of the hardware class does all the work here
         // Minimum rotational position */
        double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway
        //position robot.init(hardwareMap);
        boolean rampUp = true;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver Lookin good today");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
             servo = hardwareMap.get(Servo.class, "left_hand");
            //autonomous
            
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = gamepad1.left_stick_y;
            turn  =  -gamepad1.right_stick_x;
            //turns thing
            turns = 0;
            trim=0;
            leftArm = turns;
            //leftArm = gamepad1.right_trigger;
            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;
            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
                turns /= max;
            }

            // Output the safe vales to the motor drives.
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);
            robot.leftArm.setPower(turns);

            if (gamepad1.right_trigger > 0)
                turns = .45;
                robot.leftArm.setPower(turns);
            if (gamepad1.left_trigger>0)
                turns = -.45;
                robot.leftArm.setPower(turns);



            if (rampUp) {

                // Keep stepping up until we hit the max value.
                    position += INCREMENT ;
                if (position >= MAX_POS ) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                    }
                    }
                else {
                    // Keep stepping down until we hit the min value.
                    position -= INCREMENT ;
                    if (position <= MIN_POS ) {
                        position = MIN_POS;
                        rampUp =! rampUp;  // Switch ramp direction
                    }// Move both servos to new position.  Assume servos are mirror image of each other.
                }
            servo.setPosition(position);
            sleep(CYCLE_MS);// Send telemetry message to signify robot running;
            idle();
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("leftArm","%.2f",leftArm);
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.update();
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
