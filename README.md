# GPA_Mega

## Software for GPA Mega flight controller

![grafik](https://github.com/user-attachments/assets/2845562c-0ee8-4852-91cd-a0b50965161c)



## Install Required Software:

### ST Software
Install CubeIDE in default location i.e. `C:\ST\Stm32CubeIDE`. It is recommended to use the `C:\ST` folder for all ST programs.

*Note: CubeMX is only needed for generating the initial project structure. When using the existing Project from GitHub CubeMX is not needed.*

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)
- [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
- [STM32CubeProg](https://www.st.com/en/development-tools/stm32cubeprog.html)

It might be necessary to add `C:\ST` as a windows environment variable. If Programs weren't installed at `C:\ST` additional env. variables might be needed for the respective locations.

In CubeMX install STM32CubeH7 Firmware Package.


### Arm Toolchain
- [Arm Gnu Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (For Windows: select file ending in arm-none-eabi.exe) 

Check 'Add path to environment variable' before finishing the installation.


### Visual Studio Code
- [VisualStudioCode](https://code.visualstudio.com/download)


Install VS Code Extensions from the Extensions Manager:
+ STM32 VS Code Extension *(Only for initial project setup)*
+ Cmake tools

### Possibly other needed software
+ Cmake
+ ninja (requires manual adding to PATH)
+ Github desktop or Git





## Project Setup
*Note: This is for generating the initial project structure. When using the existing Project from GitHub CubeMX & STM32 VS Code Extension are not used.*

Start new Project or open existing Project in CubeMX.

Configure Pinout, Clock, etc.

To generate the project folder open Code Generator and enter Name/Location. 

Choose 'Toolchain: SMT32CUBEIDE'
and check 'Generate Under Root'.


Open the generated folder in VS Code. Now select the STM32 VS Code Extension and click 'Import local project'. Select the cproject file that is contained in the project folder. 

(*Start here when using the existing Project*)

Try building the project by either clicking build on the bottom left or by pressing Shift+Strg+P and writing 'cmake build'. If asked between Release & Debug, choose Debug.

Do not push the build folder as it contains hardware specific content (folder structure)

## Usage of GitHub
To enable collaborative work we use GitHub. **Right now pushing to main is fine.** Later feature branches should be used for individual areas of development.

To use this in VS Code select Source Control. Additionally GitHub desktop is recommended. Alternatively there is always git.
