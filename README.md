# Note-Tracking
Code that has active tracking with a note using a limelight and google coral

>[!Warning] 
> This code has no autonomous and no odometry only swerve
>
> Full code is in [here](https://github.com/ramondeleonca/MXTO-CLEAN)

## Limelight setup
For the limelight we just have a google coral connected via USB. And have a [note-detecting pipeline](https://docs.limelightvision.io/docs/resources/downloads#neural-networks).

## Pointing towards the Note
To point towards the note I have a simple PID that tries to get the x-offset of the note as close to 0 as possible

> [!tip]
> Dont know how swerve works? go [here](https://github.com/Blue-Ignition3526/SWERVE-DRIVE/blob/main/README.md).

## Things TODO:
- [ ] Add Images to README.md  
- [ ] Make Robot get closer to note after succsesfully centering it
