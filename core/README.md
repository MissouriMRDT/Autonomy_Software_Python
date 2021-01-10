# Core

Core contains various building blocks of the autonomy system used throughout in algorithms/interfaces. This includes: logging and RoveComm as well as telemetry and other handlers for things like waypoints.

Finally core also contains the manifest and constant files for variables that are needed across the project.

Core also contains two sub-package: **states** and **vision**

## States

The states sub-package encapsulates all the state machine specific code for autonomy this includes the state machine, state abstract class and all the individual states which are implemented as classes.

The states and state machine are written to be used using asyncio allowing for asynchronous
operation of the state machine as well as vision/sensor code. All states have a run method that performs all the expected actions in that state, and return the new state after one execution. It also defines on_event(), start() and exit() methods to handle transitions as well as scheduling/canceling tasks specific to that state.

The current state is continously run and updated by the state machine. The state machine also exposes enable and disable methods which serve as callbacks for RoveComm messages to enable/disable autonomy operation.

## Vision

The vision sub-package covers all vision components (but not algorithms!) used in the Autonomy stack. This includes the handlers for various supported cameras: ZED and Webcam (for non-obstacle avoidance). It also implements a feed handler which is responsible for spinning up seperate proccesses to stream/save video streams generated from the camera system. Videos are streamed across the network pack to Base Station, as well as saved as individual video files.
