# Team 6512 Robot Code (2025)
This is the current working code for the 2024-2025 season. It is still a work in progress, but it demonstrates our progress for using Swerve for the first time for a competition season. This is a snapshot of everything that has been worked on since kickoff.

We are using the SDS Mk4C Swerve Modules with REV Neos and Spark MAX Motor controllers. For encoders, we are using Thrifty absolute magnetic encoders and the built-in Spark Max relative encoders. Our robot also features REV CIM brushed motors for the arm, shooter, and climbing mechanisms.

After much brainstorming, our team decided to stick with a heavy influence from the current Everybot design, modified for a swerve frame. We decided to still add our own twist on a few elements to keep things original.

## Functions
An arm moves up and down with a range of around 90 degrees, sticking outside the frame in order to get Algae from the floor. To assist with this, a motor connected to a wheel mechanism is used to intake the Algae and then shoot it into the Processor. This motor is also used to intake Coral when the arm is in the up position from the human players to shoot into L1 on the Reef. For the endgame, we have a climbing mechanism connected to two different commands. Since the climbing system requires the claw to be outside the frame, one command slowly moves the claw outside the frame while keeping the arm in the up position. The other command slowly moves the claw back into the frame, essentially pulling the robot up off the ground. The swerve system uses the encoders for wheel position and a Nav-X to determine robot orientation for field-oriented mode.

## To Do
- Have the programming team analyze code to make sure all members fully understand everything and where we are starting from
- Add comments explaining everything and how it all works
- Work on more advanced stuff like kinematics and odometry for smoother driving
- Work on autonomous using PathPlanner/PathWeaver and figure out why it is not working as intended
- Work on creating more advanced autonomous commands to perform more tasks

## Created with help from these projects:
- 4534's 2023 Swerve Code - Thanks to our mentor team, the Wired Wizards, for helping us get our swerve drivetrain up and working, along with providing their swerve code.
- 2495's Edited MAXSwerve Template - Used the ThriftyEncoder class from this project, as we couldn't get the Thrifty absolute magnetic encoders working with base WPILIB libraries.
- 6624's Swerve Example - Although this code caused our drivetrain to behave strangely, 6624's swerve documentation gave us a good idea of how many of the concepts that swerve is based around worked.

## 
<img src="https://coastalcatastrophe6512.weebly.com/uploads/1/2/9/8/129892330/copy-of-coastal-catastrophe-logo_orig.png" alt="Coastal Catastrophe 6512 Logo" width="267" height="70">
