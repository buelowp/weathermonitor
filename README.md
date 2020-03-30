# Particle Photon based outdoor weather station
This is meant to track temp/humidity and listen for lightning. It's an outdoor based project for use with a particle photon, and utilizes MQTT to communicate. Every minute, it will send a temp and humidity value and every time a lightning strike is detected, it will end out a lightning event.

## Needed

One SHT10 waterproof temp/humidity sensor
One AD3935 lightning sensor
One Particle photon

## Compiling your project

When you're ready to compile your project, make sure you have the correct Particle device target selected and run `particle compile <platform>` in the CLI or click the Compile button in the Desktop IDE. The following files in your project folder will be sent to the compile service:

- Everything in the `/src` folder, including your `.ino` application file
- The `project.properties` file for your project
- Any libraries stored under `lib/<libraryname>/src`

## Potential future additions

I'm working on a PCB for this to make it easier to build. Not requireed, but why not. Then I can add an i2c grove type interface, and add support for other pluggable sensors. One thing I would like to do is add a geiger counter based on the pocket geiger solution. I may add some sort of light or UV sensor too.