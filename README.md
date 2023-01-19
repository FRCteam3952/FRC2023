# FRC 2023

Team 3952 - 2023 Season FRC robot code written in Java.

## Setup Instructions

### General
1. Clone this repo either through the download button or from the GitHub Desktop app
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options

### Visual Studio Code 
1. Get the WPILib extension from the VSCode Marketplace (requires Java 11 or greater)
1. In [`.vscode/settings.json`] set the User Setting, `java.home`, to the correct directory pointing to your downloaded JDK 11 directory

### Basic Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests