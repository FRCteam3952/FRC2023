# Troy Robotics - FRC 2023 Codebase

Team 3952's robot for the 2023 Season "FIRST Charged Up" using Java and the [WPILib](https://github.com/wpilibsuite/allwpilib) library.

## Setup Instructions
### Requirements
1. You NEED to have Java installed on your local system. Either download [Oracle JDK](https://www.oracle.com/java/technologies/downloads/#java11) or [OpenJDK](https://openjdk.org/projects/jdk/17/) 
2. You need a code editor, preferably [Visual Studio Code](https://code.visualstudio.com/) or IntelliJ IDEA
3. You must have [Git](https://git-scm.com/) installed
4. Run the [WPILib Installer](https://github.com/wpilibsuite/allwpilib/releases) this will include all the needed VSCode plugins and all of the NI software tools. Make sure to download the correct file for your operating system (Windows users download "WPILib_Windows-2023.2.1.iso" and start the install .EXE)

### Cloning
* Open a Bash shell and clone
```bash
$ git clone https://github.com/FRCteam3952/FRC2023.git
```
* OR Clone this repo either through the green download button, or from the GitHub Desktop app

## Importing
### Visual Studio Code 
1. Clone the project into a known folder location
2. The WPILib VSCode extension should automatically detect the project so you can start editing

### Basic Bash Commands
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
2. Run `./gradlew build` to build the code.  Use the `--info` flag for more details
3. To deploy code into the robot, you must first connect to the robot's Wi-Fi network, then Run `./gradlew deploy` 
4. It is recommended to run the clean task before deploying again to clear previous deployed binaries. To do this, run `./gradlew clean deploy`
* Run `./gradlew test` to run all of the JUnit tests
* Run `./gradlew tasks` to see available options

## Contributing
* Read documentation for [WPILib](https://docs.wpilib.org/en/latest/)
* Check the current [issues page](https://github.com/FRCteam3952/FRC2023/issues)
* Previous [examples](https://github.com/troyfrc3952/Basic-Robot-Code)
* Add a comment to explain what your contributed code does
* Ask for help!

## Other Code We Used 
* [FRC2023RobotGUI](https://github.com/SeanSon2005/FRC2023RobotGUI)
* [TroyFRC2023VisionCode](https://github.com/bobandjoe/TroyFRC2023VisionCode)
* [FRC2023_Vision](https://github.com/FRCteam3952/FRC2023_Vision)
