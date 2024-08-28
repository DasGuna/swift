#!/usr/bin/env python
"""
@author Jesse Haviland
"""

from os import read
import numpy as np
import spatialmath as sm
from spatialgeometry import Shape
import time
from queue import Queue, Empty
import json
from swift import start_servers, SwiftElement, Button
from swift.phys import step_v, step_shape
from typing import Union
from typing_extensions import Literal as L
from dataclasses import dataclass


@dataclass
class SwiftObject:
    obj: any = None # type: ignore
    in_sim: bool = False
    in_vis: bool = False
    is_splat: bool = False
    robot_alpha: float = 0.0
    collision_alpha: float = 0.0
    readonly: int = 0


rtb = None


def _import_rtb():  # pragma nocover
    import importlib

    global rtb
    try:
        rtb = importlib.import_module("roboticstoolbox")
    except ImportError:
        print("\nYou must install the python package roboticstoolbox-python\n")
        raise




class Swift:
    """
    Graphical backend using Swift

    Swift is a web app built on three.js. It supports many 3D graphical
    primitives including meshes, boxes, ellipsoids and lines. It can render
    Collada objects in full color.

    Examples
    --------
    .. code-block:: python
        :linenos:

        import roboticstoolbox as rtb

        robot = rtb.models.DH.Panda()  # create a robot

        pyplot = rtb.backends.Swift()   # create a Swift backend
        pyplot.add(robot)              # add the robot to the backend
        robot.q = robot.qz             # set the robot configuration
        pyplot.step()                  # update the backend and graphical view

    """

    def __init__(self, _dev=False):
        self.outq = Queue()
        self.inq = Queue()

        self._dev = _dev

        if rtb is None:
            _import_rtb()

        self._init()

    def _init(self):
        """
        A private initialization method to make relaunching easy
        """

        # This is the time that has been simulated according to step(dt)
        self.sim_time = 0.0

        # This holds all simulated objects within swift
        self.swift_objects = []
        # NEW dictionary of swift objects based on id 
        self.swift_dict = {}

        # This is an option dict with the format id: {option: option_value}
        # to hold custom options for simulated objects
        self.swift_options = {}

        # Number of custom html elements added to page for id purposes
        self.elementid = 0

        # Frame skipped to keep at rate
        self._skipped = 1

        # Element dict which holds the callback functions for form updates
        self.elements = {}

        self.headless = False
        self.rendering = True
        self._notrenderperiod = 1
        self.recording = False
        self._connected = False
        self._laststep = time.time()

    @property
    def rate(self):
        return self._rate

    @rate.setter
    def rate(self, new):
        self._rate = new
        self._period = 1 / new

    def __repr__(self):
        s = f"Swift backend, t = {self.sim_time}, scene:"

        for ob in self.swift_objects:
            s += f"\n  {ob.name}"
        return s

    #
    #  Basic methods to do with the state of the external program
    #

    def launch(
        self,
        realtime: bool = False,
        headless: bool = False,
        rate: int = 60,
        browser: Union[str, None] = None,
        open_tab: bool = False,
        comms: L["websocket", "rtc"] = "websocket",
        **kwargs,
    ):
        """
        Launch the Swift Simulator

        ``env = launch(args)`` create a 3D scene in a running Swift instance as
        defined by args, and returns a reference to the backend.

        Parameters
        ----------
        realtime
            Force the simulator to display no faster than real time, note that
            it may still run slower due to complexity
        headless
            Do not launch the graphical front-end of the simulator. Will still
            simulate the robot. Runs faster due to not needing to display
            anything.
        rate
            The rate (Hz) at which the simulator will be run, defaults to 60Hz
        browser
            browser to open in: one of 'google-chrome', 'chrome', 'firefox',
            'safari', 'opera' or see for full list
            https://docs.python.org/3.8/library/webbrowser.html#webbrowser.open_new
        comms
            The type of communication to use between the Python and Swift
            instances.  Can be either 'websocket' or 'rtc' (WebRTC).  The
            default is 'websocket'. The 'rtc' option requires a browser that
            supports WebRTC, such as Chrome or Firefox.

        """

        self.browser = browser
        self.rate = rate
        self.realtime = realtime
        self.headless = headless
        self.open_tab = open_tab

        if comms == "rtc":
            self._comms = "rtc"
        else:
            self._comms = "websocket"

        # NOTE: Currently headless makes no attempt to connect to a running static page
        # Ideally, the simulator should be agnostic of the static page (being available or not)
        # Therefore, running headless means the sim runs as expected, however, a user should still 
        # be able to open a broswer on the system and view the state of the simulation. This could also provide 
        # added bonuses regarding GPU computation (say if the simulator was running on a lower PC, but the visualisation occured on 
        # better hardware on the same network)
        if not self.headless:
            # The realtime, render and pause buttons
            # self._add_controls()

            # A flag for our threads to monitor for when to quit
            self._run_thread = True
            # NOTE: older method from SwiftRoute
            # self.socket, self.server, self.test_thread = start_servers(
            #     outq=self.outq,
            #     inq=self.inq,
            #     stop_servers=self._servers_running,
            #     socket_status=self._socket_status,
            #     test=self._test_thread,
            #     browser=browser,
            #     comms=self._comms,
            #     open_tab=self.open_tab
            # )
            # NOTE: new method from SwiftSocket
            self.socket, self.server, self.test_thread = start_servers(
                outq=self.outq,
                inq=self.inq,
                stop_servers=self._servers_running,
                socket_status=self._socket_status,
                socket_manager=self._socket_manager,
            )
            self.last_time = time.time()

    def _socket_manager(self):
        # This connected class variable is updated from the socket thread on successful connection
        # While this is false, we want to do the following:
        # - Get the connected status on a consist poll. The current implementation blocks for this when the socket thread has been created.
        # - On connection, we would want to upload any added objects (this is currently handled within one add method currently)
        # - End this thread by reaching end of scope. Ideally we re-run this in the event another connection is needed. 
        #   In these cases we would want to check if any existing objects have already been added or not
        while self._connected is False:
            time.sleep(0.1)
        
        # Clear queue on initial connection (will send back a connected from client)
        ret = self.inq.get()
        # Socket connection made!
        print(f"SWIFT: SOCKET MANAGER -> SOCKET CONNECTION IS: {self._connected} | {ret}")
        # Iterate through existing objects and handle updating while connected
        # TODO: handle removal of objects in thread
        while self._connected:
            for idx, (key, value) in enumerate(self.swift_dict.items()):
                # print(f"Available object: {value.obj} | check if added: {value.in_vis}")
                # if the object is not yet in the visualisation, serve and track
                if not value.in_vis:
                    print(f"SWIFT: SOCKET MANAGER -> {value} has not been added yet. Adding...")
                    self.serve_object(value)
                    value.in_vis = True

                # TODO: add mechanism to remove if required
            
            time.sleep(0.1)
        
        print(f"SWIFT: SOCKET MANAGER -> End of Thread Reached")

    def _socket_status(self, check: bool):
        self._connected = check 
        
    def _servers_running(self):
        return self._run_thread

    def _stop_threads(self):
        print(f"SWIFT: stopping threads...")
        self._run_thread = False
        if not self.headless:
            self.socket.join(1)
        if not self._dev:
            self.server.join(1)

        self.test_thread.join(1)

    def step(self, dt=0.05, render=True):
        """
        Update the graphical scene

        :param dt: time step in seconds, defaults to 0.05
        :type dt: int, optional
        :param render: render the change in Swift. If True, this updates the
            pose of the simulated robots and objects in Swift.
        :type dt: bool, optional

        ``env.step(args)`` triggers an update of the 3D scene in the Swift
        window referenced by ``env``.

        .. note::

            - Each robot in the scene is updated based on
              their control type (position, velocity, acceleration, or torque).
            - Upon acting, the other three of the four control types will be
              updated in the internal state of the robot object.
            - The control type is defined by the robot object, and not all
              robot objects support all control types.
            - Execution is blocked for the specified interval

        """

        # TODO how is the pose of shapes updated prior to step?

        # Update local pose of objects
        for i, obj in enumerate(self.swift_objects):
            if isinstance(obj, Shape):
                self._step_shape(obj, dt)
                obj._propogate_scene_tree()
            elif isinstance(obj, rtb.Robot):
                self._step_robot(obj, dt, self.swift_options[i]["readonly"])
                obj._propogate_scene_tree()
        # Update world transform of objects
        # for obj in self.swift_objects:
            # obj._propogate_scene_tree()

        # Adjust sim time
        self.sim_time += dt

        print(f"RUNNING => sim_time: {self.sim_time} | Connected: {self._connected}")
        if not self.headless and self._connected:

            if render and self.rendering:

                if self.realtime:
                    # If realtime is set, delay progress if we are
                    # running too quickly
                    time_taken = time.time() - self.last_time
                    diff = (dt * self._skipped) - time_taken
                    self._skipped = 1

                    if diff > 0:
                        time.sleep(diff)

                    self.last_time = time.time()
                elif (time.time() - self._laststep) < self._period:
                    # Only render at 60 FPS
                    self._skipped += 1
                    return

                self._laststep = time.time()

                # NOTE: does need connection to the socket
                self._step_elements()

                # NOTE: does need connection to the socket
                events = self._draw_all()
                # print(events)

                # Process GUI events
                self.process_events(events)

            elif not self.rendering:
                if (time.time() - self._laststep) < self._notrenderperiod:
                    return
                self._laststep = time.time()
                events = json.loads(self._send_socket("shape_poses", [], True))
                self.process_events(events)

            # print(events)
            # else:
            #     for i in range(len(self.robots)):
            #         self.robots[i]['ob'].fkine_all(self.robots[i]['ob'].q)

            self._send_socket("sim_time", self.sim_time, expected=False)

    def reset(self):
        """
        Reset the graphical scene

        ``env.reset()`` triggers a reset of the 3D scene in the Swift window
        referenced by ``env``. It is restored to the original state defined by
        ``launch()``.

        """

        self.restart()

    def restart(self):
        """
        Restart the graphics display

        ``env.restart()`` triggers a restart of the Swift view referenced by
        ``env``. It is closed and relaunched to the original state defined by
        ``launch()``.

        """

        self._send_socket("close", "0", False)
        self._stop_threads()
        self._init()
        self.launch(
            realtime=self.realtime,
            headless=self.headless,
            rate=self.rate,
            browser=self.browser,
        )

    def close(self):
        """
        Close the graphics display

        ``env.close()`` gracefully disconnectes from the Swift visualizer
        referenced by ``env``.
        """

        self._send_socket("close", "0", False)
        self._stop_threads()

    # -- NEW METHODS
    #
    #  Methods to interface with the robots created in other environemnts
    #
    def serve_object(self, swift_object: SwiftObject = None):
        """Serve objects to a connected client 
        """
        # Iterate through existing swift objects 
        # If connected, serve to client webpage (for visual display)
        if not self._connected or swift_object is None:
            return

        print(f"SWIFT: attempting add SwiftObject -> {swift_object}")
        if isinstance(swift_object.obj, Shape):
            swift_object.obj._propogate_scene_tree()
            # NOTE: need to send the already configured ID to the webpage rather than get from the client
            print(f"SHAPE params sent: {swift_object.obj.to_dict()}")
            id = int(self._send_socket("shape", [swift_object.obj.to_dict()]))

            while not int(self._send_socket("shape_mounted", [id, 1])):
                time.sleep(0.1)
        elif isinstance(swift_object.obj, SwiftElement):
            # TODO: this needs testing
            # NOTE: need to send the already configured ID to the webpage rather than get from the client
            print(f"ELEMENT params sent: {swift_object.obj.to_dict()}")
            self._send_socket("element", swift_object.obj.to_dict())
        elif isinstance(swift_object.obj, rtb.Robot):
            # Update robot transforms
            swift_object.obj._update_link_tf()
            swift_object.obj._propogate_scene_tree()
            # Update robot qlim
            swift_object.obj._qlim = swift_object.obj.qlim
            # Prepare object with expected alpha data
            robob = swift_object.obj._to_dict(
                robot_alpha=swift_object.robot_alpha, collision_alpha=swift_object.collision_alpha
            )
            # NOTE: need to send the already configured ID to the webpage rather than get from the client
            print(f"ROBOT params sent: {swift_object.obj._to_dict()}")
            id = self._send_socket("shape", robob)

            while not int(self._send_socket("shape_mounted", [id, len(robob)])):
                time.sleep(0.1)
        elif swift_object.is_splat:
            id = int(self._send_socket("shape", [swift_object.obj]))
        else:
            # Nothing to do
            pass

    # TODO: rename once finalised
    def new_add(self, ob, robot_alpha=1.0, collision_alpha=0.0, readonly=False):
        """Addition method that is agnostic of socket connection

        :param ob: _description_
        :type ob: _type_
        :param robot_alpha: _description_, defaults to 1.0
        :type robot_alpha: float, optional
        :param collision_alpha: _description_, defaults to 0.0
        :type collision_alpha: float, optional
        :param readonly: _description_, defaults to False
        :type readonly: bool, optional
        :return: _description_
        :rtype: _type_
        """
        print(f"SWIFT: entrypoint to new add method")
        if isinstance(ob, Shape):
            ob._propogate_scene_tree()
            # Check if shape has already been added 
            if ob._added_to_swift:
                print(f"SWIFT: {ob} [SHAPE] already in graphcial interface")
                return -1
            else:
                # Indicate addition to simulator (not necessarily graphical interface)
                ob._added_to_swift = True
                id = len(self.swift_objects)
                self.swift_objects.append(ob)
                # Update swift object dictionary
                self.swift_dict[int(id)] = SwiftObject(obj=ob, in_sim=True)
                return int(id)
        elif isinstance(ob, SwiftElement):
            # NOTE: This section should be set true when successfully added to client (upon connection)
            # TODO: Move this to server side add method (when ready)
            if ob._added_to_swift:
                print(f"SWIFT: {ob} [SWIFT ELEMENT] already in graphcial interface")
                return -2
            else:
                # Indicate addition to simulator (not necessarily graphical interface)
                ob._added_to_swift = True
                id = self.elementid
                self.elementid += 1
                self.elements[str(id)] = ob
                ob._id = id
                # Update swift object dictionary
                self.swift_dict[int(id)] = SwiftObject(obj=ob, in_sim=True)
                return int(id)
        elif isinstance(ob, rtb.Robot):
            # Update robot transforms
            ob._update_link_tf()
            ob._propogate_scene_tree()
            # Update robot qlim
            ob._qlim = ob.qlim
            # Update id based on list of objects
            id = len(self.swift_objects)
            self.swift_objects.append(ob)

            # Update swift object dictionary
            self.swift_dict[int(id)] = SwiftObject(
                obj=ob, 
                in_sim=True, 
                robot_alpha=robot_alpha, 
                collision_alpha=collision_alpha, 
                readonly=readonly
            )

            # TODO: update for use with new dictionary
            self.swift_options[int(id)] = {
                "robot_alpha": robot_alpha,
                "collision_alpha": collision_alpha,
                "readonly": readonly,
            }

            return int(id)
        else:
            # Currently only handling splat cases (passed in as a dict of params)
            # TODO: improve this
            if isinstance(ob, dict) and ob['stype'] == 'splat':
                self.swift_objects.append(ob)
                id = len(self.swift_objects)
                self.swift_dict[int(id)] = SwiftObject(
                    obj=ob,
                    in_sim=True,
                    is_splat=True
                )
                return int(id)
            else:
                return -3

    def add(self, ob, robot_alpha=1.0, collision_alpha=0.0, readonly=False):
        """
        Add a robot to the graphical scene

        :param ob: the object to add
        :type ob: Robot or Shape
        :param robot_alpha: Robot visual opacity. If 0, then the geometries
            are invisible, defaults to 1.0
        :type robot_alpha: bool, optional
        :param collision_alpha: Robot collision visual opacity. If 0, then
            the geometries defaults to 0.0
        :type collision_alpha: float, optional
        :param readonly: If true, swift will not modify any robot attributes,
            the robot is only being displayed, not simulated,
            defaults to False
        :type readonly: bool, optional
        :return: object id within visualizer
        :rtype: int

        ``id = env.add(robot)`` adds the ``robot`` to the graphical
            environment.

        .. note::

            - Adds the robot object to a list of robots which will be updated
              when the ``step()`` method is called.

        """
        # id = add(robot) adds the robot to the external environment. robot
        # must be of an appropriate class. This adds a robot object to a
        # list of robots which will act upon the step() method being called.
        # NOTE: addition here (if socket is not open, will mean the object is never added)
        #       future implementation should attempt to add, append (as currently) and attempt
        #       a re-addition upon connection. A possible way to segregate these is to have
        #       the higher level add (this method) simply perform the 'non-headless' function
        #       with a separate add method to update the client (once connected) with 'added' objects
        if isinstance(ob, Shape):
            ob._propogate_scene_tree()
            # NOTE: This method should be set true when successfully added to client (upon connection)
            # TODO: Move this to server side add method (when ready)
            ob._added_to_swift = True
            print(f"params sent: {ob.to_dict()}")
            # NOTE: this section to be moved to socket side addition method
            if not self.headless:
                id = int(self._send_socket("shape", [ob.to_dict()]))

                while not int(self._send_socket("shape_mounted", [id, 1])):
                    time.sleep(0.1)

            else:
                id = len(self.swift_objects)

            self.swift_objects.append(ob)
            return int(id)
        elif isinstance(ob, SwiftElement):

            # NOTE: This section should be set true when successfully added to client (upon connection)
            # TODO: Move this to server side add method (when ready)
            if ob._added_to_swift:
                raise ValueError("This element has already been added to Swift")
            ob._added_to_swift = True

            # id = 'customelement' + str(self.elementid)
            id = self.elementid
            self.elementid += 1
            self.elements[str(id)] = ob
            ob._id = id

            # NOTE: this section to be moved to socket side addition method
            self._send_socket("element", ob.to_dict())
        elif isinstance(ob, rtb.Robot):

            # if ob.base is None:
            #     ob.base = sm.SE3()

            # ob._swift_readonly = readonly
            # ob._show_robot = show_robot
            # ob._show_collision = show_collision

            # Update robot transforms
            ob._update_link_tf()
            ob._propogate_scene_tree()

            # Update robot qlim
            ob._qlim = ob.qlim
            
            print(f"SWIFT: adding robot...")

            # NOTE: this section to be moved to socket side addition method
            if not self.headless:
                robob = ob._to_dict(
                    robot_alpha=robot_alpha, collision_alpha=collision_alpha
                )
                id = self._send_socket("shape", robob)

                while not int(self._send_socket("shape_mounted", [id, len(robob)])):
                    time.sleep(0.1)

            else:
                id = len(self.swift_objects)

            self.swift_objects.append(ob)

            self.swift_options[int(id)] = {
                "robot_alpha": robot_alpha,
                "collision_alpha": collision_alpha,
                "readonly": readonly,
            }

            return int(id)

    # TEST GS IMPLEMENTATION 
    def add_splat(self, params):
        # TODO: validate the path before sending
        # TEMP Creation of dict params to send
        # TODO: add other params, such as position, orientation, etc...
        if not self.headless:
            # Send the shape function command to mount provided splat path
            id = int(self._send_socket("shape", [params]))

            # TODO: Implement a similar wait as there is some time before it has been added
            # while not int(self._send_socket("shape_mounted", [id, 1])):
                # time.sleep(0.1)

        # TODO: Implement a similar member variable to track gs requests
        # NOTE: currently only handles one gs at a time
        # self.swift_objects.append(ob)
        return int(id)

    def remove(self, id):
        """
        Remove a robot/shape from the graphical scene

        ``env.remove(robot)`` removes the ``robot`` from the graphical
            environment.

        :param id: the id of the object as returned by the ``add`` method,
            or the instance of the object
        :type id: Int, Robot or Shape
        """

        # ob to remove
        idd = None
        code = None

        if isinstance(id, rtb.ERobot) or isinstance(id, Shape):

            for i in range(len(self.swift_objects)):
                if self.swift_objects[i] is not None and id == self.swift_objects[i]:
                    idd = i
                    code = "remove"
                    self.swift_objects[idd] = None
                    break
        else:
            # Number corresponding to robot ID
            idd = id
            code = "remove"
            self.robots[idd] = None

        if idd is None:
            raise ValueError(
                "the id argument does not correspond with a robot or shape in Swift"
            )

        self._send_socket(code, idd)

    def hold(self):  # pragma: no cover
        """
        hold() keeps the browser tab open i.e. stops the browser tab from
        closing once the main script has finished.

        """

        while True:
            time.sleep(1)

    def start_recording(self, file_name, framerate, format="webm"):
        """
        Start recording the canvas in the Swift simulator

        :param file_name: The file name for which the video will be saved as
        :type file_name: string
        :param framerate: The framerate of the video - to be timed correctly,
            this should equalt 1 / dt where dt is the time supplied to the
            step function
        :type framerate: float
        :param format: This is the format of the video, one of 'webm', 'gif',
            'png', or 'jpg'
        :type format: string

        ``env.start_recording(file_name)`` starts recording the simulation
            scene and will save it as file_name once
            ``env.start_recording(file_name)`` is called
        """

        valid_formats = ["webm", "gif", "png", "jpg"]

        if format not in valid_formats:
            raise ValueError("Format can one of 'webm', 'gif', 'png', or 'jpg'")

        if not self.recording:
            self._send_socket("start_recording", [framerate, file_name, format])
            self.recording = True
        else:
            raise ValueError(
                "You are already recording, you can only record one video at a time"
            )

    def stop_recording(self):
        """
        Start recording the canvas in the Swift simulator. This is optional
        as the video will be automatically saved when the python script exits

        ``env.stop_recording()`` stops the recording of the simulation, can
            only be called after ``env.start_recording(file_name)``
        """

        if self.recording:
            self._send_socket("stop_recording")
        else:
            raise ValueError(
                "You must call swift.start_recording(file_name) before trying"
                " to stop the recording"
            )

    def screenshot(self, file_name="swift_snap"):
        """
        Save a screenshot of the current Swift frame as a png file

        :param file_name: The file name for which the screenshot will be saved as
        :type file_name: string

        ``env.screenshot(file_name)`` saves a screenshot and downloads it as file_name
        """

        if file_name.endswith(".png"):
            file_name = file_name[:-4]

        self._send_socket("screenshot", [file_name])

    def process_events(self, events):
        """
        Process the event queue from Swift, this invokes the callback functions
        from custom elements added to the page. If using custom elements
        (for example `add_slider`), use this function in your event loop to
        process updates from Swift.
        """
        # events = self._send_socket('check_elements')
        for event in events:
            self.elements[event].update(events[event])
            self.elements[event].cb(events[event])

    def set_camera_pose(self, position, look_at):
        """
        Swift.set_camera_pose(position, look_at) will set the camera
        position and orientation of the camera within the swift scene.
        The camera is located at location and is oriented to look at a
        point in space defined by look_at. Note that the camera is
        oriented with the positive z-axis.

        :param position: The desired position of the camera
        :type position: 3 vector (list or ndarray)
        :param look_at: A point in the scene in which the camera will look at
        :type look_at: 3 vector (list or ndarray)
        """

        # if isinstance(pose, sm.SE3):
        #     pose = pose.A

        # if look_at is None:
        #     q = r2q(pose[:3, :3], order="xyzs").tolist()
        # else:
        #     q = None

        if isinstance(position, np.ndarray):
            position = position.tolist()

        if isinstance(look_at, np.ndarray):
            look_at = look_at.tolist()

        transform = {
            "t": position,
            "look_at": look_at,
        }

        self._send_socket("camera_pose", transform, False)

    # NOTE: does not need connection to the socket
    def _step_robot(self, robot, dt, readonly):

        # robot._set_link_fk(robot.q)

        if readonly or robot._control_mode == "p":
            pass  # pragma: no cover

        elif robot._control_mode == "v":

            step_v(robot._n, robot._valid_qlim, dt, robot._q, robot._qd, robot._qlim)

            # _v(robot._q, robot._qd, dt, robot._qlim, robot._valid_qlim)

            # for i in range(robot.n):
            #     robot.q[i] += robot.qd[i] * (dt)

            #     if np.any(robot.qlim[:, i] != 0) and \
            #             not np.any(np.isnan(robot.qlim[:, i])):
            #         robot.q[i] = np.min([robot.q[i], robot.qlim[1, i]])
            #         robot.q[i] = np.max([robot.q[i], robot.qlim[0, i]])

        elif robot.control_mode == "a":
            pass

        else:  # pragma: no cover
            # Should be impossible to reach
            raise ValueError(
                "Invalid robot.control_mode. Must be one of 'p', 'v', or 'a'"
            )

        # Update the robot link transofrms based on the new q
        robot._update_link_tf()

    # NOTE: does need connection to the socket
    def _step_shape(self, shape, dt):

        if shape._changed:
            shape._changed = False
            id = self.swift_objects.index(shape)
            self._send_socket("shape_update", [id, shape.to_dict()])

        step_shape(
            dt, shape.v, shape._SceneNode__T, shape._SceneNode__wT, shape._SceneNode__wq
        )
        if shape.collision:
            shape._update_pyb()

        # shape._sT[:] = shape._wT @ shape._base
        # shape._sq[:] = sm.base.r2q(shape._sT[:3, :3], order="xyzs")
 
    def _step_elements(self):
        """
        Check custom HTML elements to see if any have been updated, if there
        are any updates, send them through to Swift.
        """

        for element in self.elements:
            if self.elements[element]._changed:
                self.elements[element]._changed = False
                self._send_socket(
                    "update_element", self.elements[element].to_dict(), False
                )

    def _draw_all(self):
        """
        Sends the transform of every simulated object in the scene
        Recieves bacl a list of events which has occured
        """

        msg = []

        for i in range(len(self.swift_objects)):
            if self.swift_objects[i] is not None:
                if isinstance(self.swift_objects[i], Shape):
                    msg.append([i, [self.swift_objects[i].fk_dict()]])
                elif isinstance(self.swift_objects[i], rtb.Robot):
                    msg.append(
                        [
                            i,
                            self.swift_objects[i]._fk_dict(
                                self.swift_options[i]["robot_alpha"],
                                self.swift_options[i]["collision_alpha"],
                            ),
                        ]
                    )

        events = self._send_socket("shape_poses", msg, True)
        return json.loads(events)

    def _send_socket(self, code, data=None, expected=True):
        msg = [expected, [code, data]]
        # print(f"SWIFT: sending to socket: {msg} | format: [expected, [code, data]]")
        self.outq.put(msg)

        if expected:
            ret = "0"
            try:
                ret = self.inq.get(block=True)
                # print(f"SWIFT: send socket return val: {ret}")
            except Empty:
                print(f"SWIFT: cannot connect to client")
                pass

            return ret
        else:
            return "0"

    def _pause_control(self, paused):
        # Must hold it here until unpaused
        while paused:
            time.sleep(0.1)
            events = json.loads(self._send_socket("shape_poses", []))

            if "0" in events and not events["0"]:
                paused = False
            self.process_events(events)

    def _render_control(self, rendering):
        self.rendering = rendering

    def _time_control(self, realtime):
        self._skipped = 1
        self.realtime = not realtime

    def _add_controls(self):
        self._pause_button = Button(self._pause_control)
        self._time_button = Button(self._time_control)
        self._render_button = Button(self._render_control)

        self._pause_button._id = "0"
        self._time_button._id = "1"
        self._render_button._id = "2"
        self.elements["0"] = self._pause_button
        self.elements["1"] = self._time_button
        self.elements["2"] = self._render_button
        self.elementid += 3
