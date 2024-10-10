#!/usr/bin/env python
"""
@author Jesse Haviland (original)
@author Dasun Gunasinghe (modifications)
"""
# --- IMPORTS
import copy
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
from threading import Thread, Lock
import typing, py_trees

# --- ADDITIONAL SETUP
rtb = None
def _import_rtb():  # pragma nocover
    import importlib

    global rtb
    try:
        rtb = importlib.import_module("roboticstoolbox")
    except ImportError:
        print("\nYou must install the python package roboticstoolbox-python\n")
        raise

# --- CLASSES
@dataclass
class SwiftData:
    object: any = None # type: ignore
    remove_req: bool = False
    step_req: bool = False
    in_sim: bool = False
    in_vis: bool = False
    is_splat: bool = False
    robot_alpha: float = 0.0
    collision_alpha: float = 0.0
    readonly: int = 0
    vis_id: int = 0
    vis_key: str = ''
    visible: bool = True
    visible_req: bool = False 

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
        # TODO: this should be one dictionary to track added/removed objects
        #       over a list as well as options. A data structure to contain all
        #       information (to be tracked in a dictionary) is to be finalised
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

        # -- Thread variables
        # Members as placeholders for threads (if needed and created)
        self.socket_server = None
        self.page_client = None
        self.socket_manager = None
        # Lock
        self.lock = Lock()
        self.headless = False
        self.rendering = True
        self._notrenderperiod = 1
        self.recording = False
        self._vis_running = False
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
            print(f"SWIFT: Setting up Socket...")
            self.socket_server, self.page_client, self.socket_manager = start_servers(
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

        # Acquire the shared visualiser running flag 
        self.lock.acquire()
        _vis_running = self._vis_running
        self.lock.release()

        # Confirm if running or not to validate connection
        if not _vis_running:
            print(f"SWIFT: SOCKET MANAGER -> ESTABLISHING NEW SOCKET...")

            # Wait for connection to establish correctly
            while True:
                self.lock.acquire()
                _vis_running = self._vis_running
                self.lock.release()
                
                if _vis_running is False:
                    break
                time.sleep(0.1)
            
            # Clear queue on initial connection (will send back a connected from client)
            self.inq.get()
        else:
            print(f"SWIFT: SOCKET MANAGER -> SOCKET CONNECTION ALREADY ESTABLISHED")

        # Socket connection made!
        print(f"SWIFT: SOCKET MANAGER -> SOCKET CONNECTION IS: {self._vis_running}")
        
        # Iterate through existing objects and handle updating while connected
        # TODO: handle removal of objects in thread
        while True:
            # Exit condition based on checking main swift visualiser running member
            self.lock.acquire()
            _vis_running = self._vis_running
            _swift_dict = self.swift_dict
            self.lock.release()

            if not _vis_running:
                break

            # Handle race conditions on user update (via remove call)
            removal_key_list = []
            for key in list(_swift_dict):
                # print(f"Handing Key {key}")
                # If the object is not yet in the visualisation, serve and track
                if not _swift_dict[key].in_vis:
                    self.visualiser_add_object(_swift_dict[key], key)
                    _swift_dict[key].in_vis = True

                # If the object is flagged for removal, remove from client
                if _swift_dict[key].remove_req:
                    # remove the object from the simulator
                    self.visualiser_remove_object(_swift_dict[key])
                    # remove the object from the swift dictionary data
                    removal_key_list.append(key)

                if _swift_dict[key].step_req:
                    # print(f"Key {key} requires step")
                    self.visualiser_step_object(_swift_dict[key], key)
                    _swift_dict[key].step_req = False

                if _swift_dict[key].visible_req:
                    self.visualiser_visible_update(_swift_dict[key], key)
                    _swift_dict[key].visible_req = False

                # TODO: update state (from visualiser input)
                # Implement a get shape poses method here for updating object state
            
            # Remove from swift dict
            for key in removal_key_list:
                del _swift_dict[key]

            # Update swift dict from local copy
            self.lock.acquire()
            self._swift_dict = _swift_dict
            self.lock.release()

            time.sleep(0.1)
        
        print(f"SWIFT: SOCKET MANAGER -> End of Thread Reached")

    def _socket_status(self, check: bool):
        """Thread call to update the visualiser running method

        :param check: _description_
        :type check: bool
        """
        with self.lock:
            self._vis_running = check 
        
    def _servers_running(self):
        """Thread call to return swift's higher-level call to stop function

        :return: _description_
        :rtype: _type_
        """
        with self.lock:
            return self._run_thread

    def _stop_threads(self):
        print(f"SWIFT: stopping threads...")
        self._run_thread = False
        if self.socket_server is not None and self.socket_server.is_alive():
            print(f"SWIFT: stopping {self.socket_server.name}")
            self.socket_server.join(1)
        if self.page_client is not None and self.page_client.is_alive():
            print(f"SWIFT: stopping {self.page_client.name}")
            self.page_client.join(1)
        if self.socket_manager is not None and self.socket_manager.is_alive():
            print(f"SWIFT: stopping {self.socket_manager.name}")
            self.socket_manager.join(1)

    def objects_loaded(self) -> bool:
        count = 0
        for key in self.swift_dict.keys():
            if self.swift_dict[key].in_vis:
                count+=1
        
        if count == len(self.swift_dict):
            return True
        else:
            return False

    def set_visible(self, key: str, visible: bool = True):
        self.swift_dict[key].visible = visible
        self.swift_dict[key].visible_req = True

    def new_step(self, dt=0.05):
         
        # Check if the provided ID is in the configured key list for the dictionary of data
        for key in self.swift_dict.keys():
            if isinstance(self.swift_dict[key].object, Shape):
                self._step_shape(self.swift_dict[key].object, dt)
                self.swift_dict[key].object._propogate_scene_tree()
            elif isinstance(self.swift_dict[key].object, rtb.Robot):
                self._step_robot(self.swift_dict[key].object, dt, self.swift_dict[key].readonly)
                self.swift_dict[key].object._propogate_scene_tree()

            self.swift_dict[key].step_req = True
        
        # Adjust sim time
        self.sim_time += dt


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
        # New implementation with main data dictionary of objects
        for key in self.swift_dict.keys():
            if isinstance(self.swift_dict[key].object, Shape):
                self._step_shape(self.swift_dict[key].object, dt)
            elif isinstance(self.swift_dict[key].object, rtb.Robot):
                self._step_robot(self.swift_dict[key].object, dt, self.swift_dict[key].readonly)

        for key in self.swift_dict.keys():
            self.swift_dict[key].object._propogate_scene_tree()

        # Adjust sim time
        self.sim_time += dt

        # print(f"RUNNING => sim_time: {self.sim_time} | Connected: {self._vis_running}")
        # NOTE: only run if not headless and the visualiser is successfully launched and all objects have been served
        if not self.headless and self._vis_running and self.objects_loaded():

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

                # Process GUI events
                self.process_events(events)

            elif not self.rendering:
                if (time.time() - self._laststep) < self._notrenderperiod:
                    return
                self._laststep = time.time()
                events = json.loads(self._send_socket("shape_poses", [], True))
                self.process_events(events)

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
    def visualiser_add_object(self, swift_data: SwiftData = None, key: str = None):
        """Serve objects to a connected client 
        """
        # Handle error on serve (if the visualiser is not running or data is empty)
        if not self._vis_running or swift_data is None:
            return

        # Iterate through existing swift objects 
        # If connected, serve to client webpage (for visual display)
        if isinstance(swift_data.object, Shape):
            swift_data.object._propogate_scene_tree()

            data_dict: dict = swift_data.object.to_dict()
            data_dict['uuid'] = key
            # Convert colour to hex string value for visualisation
            data_dict['color'] = f"#{('%x' % data_dict['color'])}"

            print(f"object: {data_dict}")
            vis_id = int(self._send_socket("shape", [data_dict]))

            # Wait for mount of object in visualiser
            while not int(self._send_socket("shape_mounted", [vis_id, 1])):
                time.sleep(0.1)
            
            # Finalise id in object
            swift_data.vis_id = vis_id
        elif isinstance(swift_data.object, SwiftElement):
            # TODO: this needs testing
            # NOTE: need to send the already configured ID to the webpage rather than get from the client
            # print(f"ELEMENT params sent: {swift_data.object.to_dict()}")
            data_dict: dict = swift_data.object.to_dict()
            data_dict['uuid'] = key
            # Convert colour to hex string value for visualisation
            data_dict['color'] = f"#{('%x' % data_dict['color'])}"

            self._send_socket("element", data_dict)
        elif isinstance(swift_data.object, rtb.Robot):
            # Update robot transforms
            swift_data.object._update_link_tf()
            swift_data.object._propogate_scene_tree()
            # Update robot qlim
            swift_data.object._qlim = swift_data.object.qlim
            # Prepare object with expected alpha data
            robob = swift_data.object._to_dict(
                robot_alpha=swift_data.robot_alpha, 
                collision_alpha=swift_data.collision_alpha
            )
            # Only provide key onto base mesh object in list (for visual selection)
            robob[0]['uuid'] = key
            # Convert colour to hex string value for visualisation
            for mesh in robob:
                mesh['color'] = f"#{('%x' % mesh['color'])}"

            # NOTE: need to send the already configured ID to the webpage rather than get from the client
            vis_id = self._send_socket("shape", robob)

            # Wait for mount of object in visualiser
            while not int(self._send_socket("shape_mounted", [vis_id, len(robob)])):
                time.sleep(0.1)

            # Finalise id in object
            swift_data.vis_id = vis_id
        elif swift_data.is_splat:
            # TODO: handle this as an object - currently only the params are sent (which is wrongly set as an 'object')
            data_dict: dict = swift_data.object
            data_dict['uuid'] = key
            vis_id = int(self._send_socket("shape", [data_dict]))

            # Finalise id in object
            swift_data.vis_id = vis_id
        else:
            # Nothing to do
            pass
    
    def visualiser_remove_object(self, swift_data: SwiftData = None):
        # Handle error on remove
        if swift_data is None:
            return
        
        if isinstance(swift_data.object, rtb.Robot) or isinstance(swift_data.object, Shape):
            self._send_socket(code="remove",data=swift_data.vis_id)

    def visualiser_step_object(self, swift_data: SwiftData = None, key: str = None):
        if swift_data is None or key is None:
            return

        msg = []

        # New method using the dictionary
        if swift_data.object is not None:
            if isinstance(swift_data.object, Shape):
                msg.append([key, [swift_data.object.fk_dict()]])
            elif isinstance(swift_data.object, rtb.Robot):
                msg.append([
                    key,
                    swift_data.object._fk_dict(
                        swift_data.robot_alpha,
                        swift_data.collision_alpha,
                    )
                ])

        self._send_socket("shape_poses", msg)

    def visualiser_visible_update(self, swift_data: SwiftData = None, key: str = None):
        if swift_data is None or key is None:
            return

        if swift_data.object is not None:
            print(f"Sending Shape Visible")
            self._send_socket(code="shape_visible", data=[key, swift_data.visible])

    def update_tree(self, 
            snapshot_visitor: py_trees.visitors.DisplaySnapshotVisitor, 
            btree: py_trees.trees.BehaviourTree):
        # From: https://py-trees.readthedocs.io/en/devel/demos.html#py_trees.demos.logging.create_tree
        # # Handle error on type
        if not isinstance(btree, py_trees.trees.BehaviourTree):
            print(f"[SWIFT] -> Error, btree not a vaild BehaviourTree type")
            return
        if not isinstance(snapshot_visitor, py_trees.visitors.DisplaySnapshotVisitor):
            print(f"[SWIFT] -> Error, snapshot_visitor not a vaild DisplaySnapshotVisitor type")
            return
        
        x_pad = 10
        x_val = 0
        y_val = 0
        y_pad = 20
        if snapshot_visitor.changed:
            tree_serialisation = {"tick": btree.count, "nodes": [], "connections": []}
            for node in btree.root.iterate():
                node_type_str = "Behaviour"
                for behaviour_type in [
                    py_trees.composites.Sequence,
                    py_trees.composites.Selector,
                    py_trees.composites.Parallel,
                    py_trees.decorators.Decorator,
                    ]:
                    if isinstance(node, behaviour_type):
                        node_type_str = behaviour_type.__name__

                if node.tip() is not None:
                    node_tip = node.tip()
                    assert node_tip is not None 
                    node_tip_id = str(node_tip.id)
                else:
                    node_tip_id = "none"

                node_snapshot = {
                    "name": node.name,
                    "id": str(node.id),
                    "parent_id": str(node.parent.id) if node.parent else "none",
                    "child_ids": [str(child.id) for child in node.children],
                    "tip_id": node_tip_id,
                    "class_name": str(node.__module__) + "." + str(type(node).__name__),
                    "type": node_type_str,
                    "status": node.status.value,
                    "message": node.feedback_message,
                    "is_active": True if node.id in snapshot_visitor.visited else False,
                }
                print(f"NODE: {node.name}")
                node_snapshot
                swift_snapshot = {
                    'id': str(node.id),
                    'data': { 'label': node.name },
                    'position': { 'x': x_val, 'y': y_val},
                    "is_active": 1 if node.id in snapshot_visitor.visited else 0,
                }
                for child in node.children:
                    print(f"child: {child.name}")
                    connection = { 
                      'id': str(node.id) + str(child.id), 
                      'source': str(node.id), 
                      'target': str(child.id) 
                    }
                    typing.cast(list, tree_serialisation["connections"]).append(connection)

                
                x_val += x_pad
                y_val += y_pad
                # if connections:
                    # typing.cast(list, tree_serialisation["connections"]).append(connections)
                typing.cast(list, tree_serialisation["nodes"]).append(swift_snapshot)
            print("----")
            print(tree_serialisation)
            print("!!!!")


    # TODO: rename once finalised
    def add(self, ob, uuid: str = None, robot_alpha: float = 1.0, collision_alpha: float = 0.0, readonly: bool = False):
        """Addition method that is agnostic of socket connection

        :param ob: _description_
        :type ob: _type_
        :param uuid: A string unique identifier
        :type uuid: str 
        :param robot_alpha: _description_, defaults to 1.0
        :type robot_alpha: float, optional
        :param collision_alpha: _description_, defaults to 0.0
        :type collision_alpha: float, optional
        :param readonly: _description_, defaults to False
        :type readonly: bool, optional
        :return: configured id (if successful).  
        :rtype: int > 0 if successful, or < 0 if in error 
        """
        # Testing error case prior to general addition
        for key in self.swift_dict.keys():
            if id(ob) == id(self.swift_dict[key].object):
                print(f"SWIFT: {id(ob)} unique ID already added to captured dictionary {id(self.swift_dict[key].object)}")
                print(f"SWIFT: Ignoring as object is already in swift...")
                return -1 
            
        # print(f"SWIFT: entrypoint to new add method")
        if isinstance(ob, Shape):
            ob._propogate_scene_tree()
            # id = len(self.swift_objects)
            self.swift_objects.append(ob)
            # Update swift object dictionary
            swift_id = len(self.swift_dict)
            self.swift_dict[uuid if uuid else int(swift_id)] = SwiftData(object=ob, in_sim=True)
        elif isinstance(ob, SwiftElement):
            swift_id = self.elementid
            self.elementid += 1
            self.elements[str(swift_id)] = ob
            ob._id = swift_id
            # Update swift object dictionary
            self.swift_dict[uuid if uuid else int(swift_id)] = SwiftData(object=ob, in_sim=True)
        elif isinstance(ob, rtb.Robot):
            # Update robot transforms
            ob._update_link_tf()
            ob._propogate_scene_tree()
            # Update robot qlim
            ob._qlim = ob.qlim
            # Update id based on list of objects
            self.swift_objects.append(ob)
            swift_id = len(self.swift_dict)
            # Update swift object dictionary
            self.swift_dict[uuid if uuid else int(swift_id)] = SwiftData(
                object=ob, 
                in_sim=True, 
                robot_alpha=robot_alpha, 
                collision_alpha=collision_alpha, 
                readonly=readonly
            )
        else:
            # Currently only handling splat cases (passed in as a dict of params)
            # TODO: improve this
            if isinstance(ob, dict) and ob['stype'] == 'splat':
                swift_id = len(self.swift_dict)
                self.swift_dict[uuid if uuid else int(swift_id)] = SwiftData(object=ob, in_sim=True, is_splat=True)
            else:
                return -2

        return uuid if uuid else swift_id

    # KEPT AS LEGACY
    # def old_add(self, ob, robot_alpha=1.0, collision_alpha=0.0, readonly=False):
    #     """
    #     Add a robot to the graphical scene

    #     :param ob: the object to add
    #     :type ob: Robot or Shape
    #     :param robot_alpha: Robot visual opacity. If 0, then the geometries
    #         are invisible, defaults to 1.0
    #     :type robot_alpha: bool, optional
    #     :param collision_alpha: Robot collision visual opacity. If 0, then
    #         the geometries defaults to 0.0
    #     :type collision_alpha: float, optional
    #     :param readonly: If true, swift will not modify any robot attributes,
    #         the robot is only being displayed, not simulated,
    #         defaults to False
    #     :type readonly: bool, optional
    #     :return: object id within visualizer
    #     :rtype: int

    #     ``id = env.add(robot)`` adds the ``robot`` to the graphical
    #         environment.

    #     .. note::

    #         - Adds the robot object to a list of robots which will be updated
    #           when the ``step()`` method is called.

    #     """
    #     # id = add(robot) adds the robot to the external environment. robot
    #     # must be of an appropriate class. This adds a robot object to a
    #     # list of robots which will act upon the step() method being called.
    #     # NOTE: addition here (if socket is not open, will mean the object is never added)
    #     #       future implementation should attempt to add, append (as currently) and attempt
    #     #       a re-addition upon connection. A possible way to segregate these is to have
    #     #       the higher level add (this method) simply perform the 'non-headless' function
    #     #       with a separate add method to update the client (once connected) with 'added' objects
    #     if isinstance(ob, Shape):
    #         ob._propogate_scene_tree()
    #         # NOTE: This method should be set true when successfully added to client (upon connection)
    #         # TODO: Move this to server side add method (when ready)
    #         ob._added_to_swift = True
    #         print(f"params sent: {ob.to_dict()}")
    #         # NOTE: this section to be moved to socket side addition method
    #         if not self.headless:
    #             id = int(self._send_socket("shape", [ob.to_dict()]))

    #             while not int(self._send_socket("shape_mounted", [id, 1])):
    #                 time.sleep(0.1)

    #         else:
    #             id = len(self.swift_objects)

    #         self.swift_objects.append(ob)
    #         return int(id)
    #     elif isinstance(ob, SwiftElement):

    #         # NOTE: This section should be set true when successfully added to client (upon connection)
    #         # TODO: Move this to server side add method (when ready)
    #         if ob._added_to_swift:
    #             raise ValueError("This element has already been added to Swift")
    #         ob._added_to_swift = True

    #         # id = 'customelement' + str(self.elementid)
    #         id = self.elementid
    #         self.elementid += 1
    #         self.elements[str(id)] = ob
    #         ob._id = id

    #         # NOTE: this section to be moved to socket side addition method
    #         self._send_socket("element", ob.to_dict())
    #     elif isinstance(ob, rtb.Robot):

    #         # if ob.base is None:
    #         #     ob.base = sm.SE3()

    #         # ob._swift_readonly = readonly
    #         # ob._show_robot = show_robot
    #         # ob._show_collision = show_collision

    #         # Update robot transforms
    #         ob._update_link_tf()
    #         ob._propogate_scene_tree()

    #         # Update robot qlim
    #         ob._qlim = ob.qlim
            
    #         print(f"SWIFT: adding robot...")

    #         # NOTE: this section to be moved to socket side addition method
    #         if not self.headless:
    #             robob = ob._to_dict(
    #                 robot_alpha=robot_alpha, collision_alpha=collision_alpha
    #             )
    #             id = self._send_socket("shape", robob)

    #             while not int(self._send_socket("shape_mounted", [id, len(robob)])):
    #                 time.sleep(0.1)

    #         else:
    #             id = len(self.swift_objects)

    #         self.swift_objects.append(ob)

    #         self.swift_options[int(id)] = {
    #             "robot_alpha": robot_alpha,
    #             "collision_alpha": collision_alpha,
    #             "readonly": readonly,
    #         }

    #         return int(id)

    # TODO: Test and validate solution
    def remove(self, id = None):
        """Remove a robot/shape from graphical scene and simulator using ID

        :param object: _description_
        :type object: _type_
        """
        # Handle error condition if no id was provided
        if id is None:
            return
        
        # ID takes precedence in removal (as multiple objects may be of the same type)
        # Check if the provided ID is in the configured key list for the dictionary of data
        if id in self.swift_dict.keys():
            # Request removal from dictionary in running socket thread
            self.swift_dict[id].remove_req = True
        else:
            print(f"SWIFT: No such id in Swift -> {id}")
            print(f"SWIFT: Current ids -> {self.swift_dict.keys()}")

    # LEGACY KEPT
    # def remove(self, id):
    #     """
    #     Remove a robot/shape from the graphical scene
    #
    #     ``env.remove(robot)`` removes the ``robot`` from the graphical
    #         environment.
    #
    #     :param id: the id of the object as returned by the ``add`` method,
    #         or the instance of the object
    #     :type id: Int, Robot or Shape
    #     """
    #
    #     # ob to remove
    #     idd = None
    #     code = None
    #
    #     if isinstance(id, rtb.ERobot) or isinstance(id, Shape):
    #
    #         for i in range(len(self.swift_objects)):
    #             if self.swift_objects[i] is not None and id == self.swift_objects[i]:
    #                 idd = i
    #                 code = "remove"
    #                 self.swift_objects[idd] = None
    #                 break
    #     else:
    #         # Number corresponding to robot ID
    #         idd = id
    #         code = "remove"
    #         self.robots[idd] = None
    #
    #     if idd is None:
    #         raise ValueError(
    #             "the id argument does not correspond with a robot or shape in Swift"
    #         )
    #
    #     self._send_socket(code, idd)

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

    # NOTE: does not need connection to the socket
    def _step_shape(self, shape, dt):

        # if shape._changed:
        #     print(f"In _step_shape -> {shape} changed")
        #     shape._changed = False
        #     id = self.swift_objects.index(shape)
        #     self._send_socket("shape_update", [id, shape.to_dict()])

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
        Recieves back a list of events which has occured
        """

        msg = []

        # New method using the dictionary
        for key in self.swift_dict.keys():
            if self.swift_dict[key].object is not None:
                if isinstance(self.swift_dict[key].object, Shape):
                    msg.append([key, [self.swift_dict[key].object.fk_dict()]])
                elif isinstance(self.swift_dict[key].object, rtb.Robot):
                    msg.append([
                        key,
                        self.swift_dict[key].object._fk_dict(
                            self.swift_dict[key].robot_alpha,
                            self.swift_dict[key].collision_alpha,
                        )
                    ])

        events = self._send_socket("shape_poses", msg, True)
        return json.loads(events)

    def _send_socket(self, code, data=None, expected=True):
        # Error handling on connection
        if not self._vis_running:
            return "0"

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
