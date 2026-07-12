# Vendored source

The files in this package (`com.arcrobotics.ftclib.command` and `.button`) are copied
from ftclib's published `core-2.1.1-sources.jar` (https://github.com/FTCLib/FTCLib,
BSD-3-Clause / FIRST BSD license per each file's own header).

They're vendored as plain source rather than pulled in as a Gradle dependency because
`org.ftclib.ftclib:core` is published as an Android `.aar`, which a plain `java-library`
module (this one) cannot consume. Only the files with no Android/FTC-hardware
dependency were copied -- `CommandOpMode`, `GamepadButton`, `LogCatCommand`,
`MecanumControllerCommand`, `OdometrySubsystem`, `ProfiledPIDCommand`,
`PurePursuitCommand`, `RamseteCommand`, `TrapezoidProfileCommand`, and the legacy
`old/` package were left out. `SelectCommand` was also left out -- its no-match
fallback path constructs a `LogCatCommand` (Android `Log`-only), and nothing in this
project uses `SelectCommand`.

`@NonNull` was stripped from `RunCommand.java` (the only remaining file that used it)
to avoid pulling in `androidx.annotation` for a compile-time-only marker with no
behavioral effect.

`WaitCommand.java` is NOT a verbatim copy -- ftclib's version uses `Timing.Timer`,
which needs `com.qualcomm.robotcore.util.ElapsedTime` (FTC RobotCore, Android-only).
Reimplemented here with plain `System.nanoTime()` instead; same behavior (does
nothing, ends after the given duration), same public API.
