# RZV2H Multi Stream Dmoe
The pupose of the projects is to examine and verify each of the software and hardware components on the RZV2H/N.

## CMAKE 
```bash
export SDK=/opt/poky/rzv2h
cmake -DCMAKE_TOOLCHAIN_FILE=./toolchain/runtime.cmake ..

source /opt/poky/3.1.26/environment-setup-aarch64-poky-linux
cmake -DCMAKE_TOOLCHAIN_FILE=./toolchain/runtime.cmake -DTVM_ENABLE=ON ..
```
## Configure DEBUG
# Setup EVK
*requirements*
- Installed gdbserver on the EVK. This a yocto rebuild.
- 
# Single Applicaiton
This is configured to workspaces(folders) with single project.
- Step 1: Open .vscode settings.json
- Step 2: Change name of name of program `"PROGRAM":"rzv2h_multi_stream_app"` to the name of the generated executable
- Step 3: Change IP address to match the IP address of the EVK baord. NOTE : Connect ehternet between Host Computer and EVK.
- Step 6: Optional: Modify the var-deploy-gdb-sh to send additional files to the EVK server
- 
# Multiple Projects
This section modifies the Single Application to support selection of different projects.
This debug setup is useful for workspaces with multiple projects.
- Step 1: Modifiy the settnigs.json file add `"APPDIR":"project1/exe",` that specifies the project executable directory.
    - For each project add additional 
  ```json
        "APPDIR1":"project1/exe",
        "APPDIR2":"project2/exe",
        "APPDIR3":"project3/exe",
  ```
- Step 2: Modify the launch.json file loacated in .vscode directory.
  - Duplicate the configurations in the loaunch.json file for each project
  - Change the configuration name to the name of the project
  - Change the `"preLaunchTask": "var-deploy-gdb",` to "preLaunchTask": `"var-deploy-gdb_0"`. Do this for each project.
- Step 3: Modifify each task.
  - For each task configuration`"label": "var-deploy-gdb",`, change the label name to reflect the `preLaunchTask` name specified in **step 3**.
  - Modifiy the tasks.json file add `"${config:VARISCITE.APPDIR}"` to the args. this will specify the project directory exe directory. 
  ```json
  "args": [
            "var-deploy-gdb.sh",
            "${config:VARISCITE.TARGET_IP}",
            "${config:VARISCITE.PROGRAM}",
            "${config:VARISCITE.APPDIR}"
        ],
  ```
  - For each of the task configuration specify `"${config:VARISCITE.APPDIR}"` to use the name defined in Step 2. 
    - i.e. `"${config:VARISCITE.APPDIR}"` -> `"${config:VARISCITE.APPDIR1}"`

  

