# effinhover ground station

Simple python interface to the hovercraft over WiFi.
Implements a simple PID loop to control heading rate, and provides a keyboard interface.

## Required libraries

In order to run the code of the ground station the following libraries are required:

- **protobuf**: Download using the terminal and a package manager. 

    Using conda
    ```
    conda install protobuf
    ```
    Using pip
    ```
    pip install protobuf
    ```
- **pygame** : follow instructions on [official website](https://www.pygame.org/wiki/GettingStarted).

## Communicating with hovercraft

- Create a WiFi with the name *hovercraft* and the password *1234abcd*. Most conveniently this can be done on your smartphone using a mobile hotspot. 
- Connect your laptop to this WiFi.
- Open a terminal in the folder *effinhover/ground* and execute the python script with
    ```
    python effinhover.py
    ```
- To terminate the script press `ctrl + C`

## Keyboard interface

| Key           | Action       | 
| :-----------: |:-------------:| 
| `a`     | Start/stop motors | 
|`x`| Increase lift|
|`z`| Decrease lift|
| `arrow up`     | Increase thrust      | 
| `arrow down`| Decrease thrust     |
| `arrow left` | Turn/rotate left |
| `arrow right`| Turn/rotate right| 
|`space`| Set rotation rate to zero|
