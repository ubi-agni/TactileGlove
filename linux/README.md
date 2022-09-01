# Tactile Glove software

## Required Dependencies

- `boost::program_options`
- `boost::thread`
- `boost::regex`
- `libqt5serialport5-dev`
- [tactile_filters](https://github.com/ubi-agni/tactile_filters)

## Optional Dependencies

- Qt5 with `libqt5svg5-dev` (for visualization)
- `ncurses` (for nice console output)
- package `tactile_msgs` from [tactile_toolbox](https://github.com/ubi-agni/tactile_toolbox) for ROS publisher support


## Build Instructions

A [`package.xml`](package.xml) is provided to allow simple integration within a ROS workspace. Otherwise, just build with cmake as follows:

```bash
mkdir build
cd build
cmake -Dtactile_filters_DIR=<path> <src path>
make
make install
```
## Usage

The software is split into two parts: a console program and a GUI program (in corresponding sub dirs).

### gloveConsole

The console program supports two output modes
- console
- ROS topic `/TactileGlove`

and two input modes
- serial (real glove connected via USB)
- dummy  (provide random dummy values)

### GloveViz GUI

The GUI program correspondingly supports several input modes that are chosen on the command line:
- serial (real glove connected via USB)
- ROS topic `/TactileGlove`
- dummy  (provide random dummy values)

If no ROS/catkin environment is found during building, ROS support will be disabled.

As the glove comes in different variants, it's possible to choose the taxel layout
as well as the mapping of taxels to individual data channels.
To this end, the GloveViz GUI uses svg files describing the layout and a taxel.cfg file
describing the mapping from svg node IDs to data channels. Both files are compiled into the binary.

As a glove layout might exist in different instantiations with different mappings,
the taxel.cfg file contains several sections, one for each known glove instantiation/mapping, each referencing the corresponding layout. Hence, on the command line, you should simply choose your glove version with the option `-m version`.

However, you can also specify your own or modify the existing mapping on the command line, either loading your own mapping file with option `-f mapping.ini` or specifying individual mapping like so: `name=channel`.

## Calibration

Both programs, `gloveConsole` and `GloveViz`, can use a calibration to map raw sensor readings (in the range of 0-4095) to real forces. An example calibration file for P2 layout is provided in `console/P2.calib.yaml`.
`gloveConsole` will publish to the topic `/TactileGlove/calibrated` in this case.

`GloveViz` maps the output range of the calibration (or 0..4095) onto colors black … green … yellow … red.
