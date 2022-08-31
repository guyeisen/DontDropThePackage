"""THIS IS SOLVER_VIEWER_MAIN WRITTEN BY MICHAEL.
    WE ONLY EDITED IT A BIT TO GET OUR NEEDS"""

import os
import sys
import json
import importlib.util
from inspect import isclass
from typing import List

from PyQt5 import QtWidgets
from discopygal.gui import RDisc

sys.path.append('./src')

from discopygal.solvers import *
from discopygal.solvers.rrt import *
from discopygal.gui.logger import Writer
from discopygal.gui.Worker import Worker
from discopygal.solvers.verify_paths import verify_paths
from discopygal.geometry_utils.display_arrangement import display_arrangement

from solver_viewer_gui import Ui_MainWindow, Ui_dialog, About_Dialog


WINDOW_TITLE = "DiscoPygal Solver Viewer"
DEFAULT_ZOOM = 30
DEFAULT_SCENE = "small_scene.json"
DEFAULT_SOLVER = "prm.py"
DEFAULT_DISC_SIZE = 0.01

def get_available_solvers():
    """
    Return a list of all available solvers' names
    """
    solver_list = []
    for obj in globals():
        if isclass(globals()[obj]) and issubclass(globals()[obj], Solver) and globals()[obj] is not Solver:
            solver_list.append(obj)
    return solver_list


class SolverDialog(Ui_dialog):
    def __init__(self, gui, dialog, default=False):
        super().__init__()
        self.gui = gui  # pointer to parent gui
        self.dialog = dialog
        self.setupUi(self.dialog)

        self.update_combo_box()
        self.selectButton.clicked.connect(self.choose_solver)
        self.browseButton.clicked.connect(self.solver_from_file)

    def solver_from_file(self, default=False):
        """
        Choose a solver from a file
        """
        if default:
            print(f"Getting default solver: {DEFAULT_SOLVER}")
            path = DEFAULT_SOLVER
        else:
            path, _ = QtWidgets.QFileDialog.getOpenFileName(
                self.dialog, 'Load File')
            if path == '':
                return
        try:
            spec = importlib.util.spec_from_file_location(path, path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            cnt = 0
            for obj_name in dir(module):
                obj = getattr(module, obj_name)
                if isclass(obj) and issubclass(obj, Solver) and obj_name != "Solver":
                    print(f"Found solver: {obj_name}")
                    globals()[obj_name] = obj
                    cnt += 1
                    break
            self.update_combo_box(default=default, default_text=obj_name)
            msgbox = QtWidgets.QMessageBox(
                QtWidgets.QMessageBox.Icon.Information, 
                "Import solvers", "Successfully import {} solvers from {}.".format(cnt, path))
            if not default:
                msgbox.exec()
        except Exception as e:
            msgbox = QtWidgets.QMessageBox(
                QtWidgets.QMessageBox.Icon.Critical, 
                "Could not import module", repr(e))
            msgbox.exec()

    def choose_solver(self):
        print(f"Choosing solver: {self.solverComboBox.currentText()}")
        self.gui.select_solver(self.solverComboBox.currentText())
        self.dialog.close()

    def update_combo_box(self, default=False, default_text=None):
        items = ["Select a solver..."] + get_available_solvers()
        self.solverComboBox.clear()
        self.solverComboBox.addItems(items)
        if default:
            self.solverComboBox.setCurrentText(default_text)
            print(f"Set current text of solver dialog to {default_text}")


class SolverViewerGUI(Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.set_program_name(WINDOW_TITLE)

        # Fix initial zoom
        self.zoom = DEFAULT_ZOOM
        self.redraw()

        # Disable scene path edit
        self.scenePathEdit.setEnabled(False)

        # Setup the scene
        self.discopygal_scene = Scene()
        self.scene_drawer = SceneDrawer(self, self.discopygal_scene)
        self.scene_path = ""
        self.actionOpen_Scene.triggered.connect(self.load_scene)
        self.actionOpenScene.triggered.connect(self.load_scene)
        self.actionClear.triggered.connect(self.clear)

        # Setup solver
        self.solver_dialog = None
        self.solver_class = None
        self.solver = None
        self.actionOpenSolver.triggered.connect(self.load_solver)
        self.actionOpen_Solver.triggered.connect(self.load_solver)
        self.actionSolve.triggered.connect(self.solve)
        self.solver_gui_elements = {}
        self.solver_graph = None
        self.solver_arrangement = None
        self.solver_graph_vertices = [] # gui
        self.solver_graph_edges = [] # gui

        # Setup concurrency
        self.threadpool = QtCore.QThreadPool()
        self.worker = None
        self.writer = Writer(self.textEdit)

        # Solution paths and misc data
        self.paths = None
        self.paths_optimized = None
        self.path_vertices = []  # gui
        self.path_vetices_optimized = []
        self.path_edges = []  # gui
        self.path_edges_optimized = []
        self.set_animation_finished_action(self.anim_finished)
        
        # Setup actions
        self.actionShowPaths.triggered.connect(self.toggle_paths)
        self.actionPlay.triggered.connect(self.animate_paths)
        self.actionPause.triggered.connect(self.action_pause)
        self.actionStop.triggered.connect(self.action_stop)
        self.actionShowGraph.triggered.connect(self.action_display_graph)
        self.actionShowArrangement.triggered.connect(self.action_display_arrangement)
        self.actionAbout.triggered.connect(self.about_dialog)
        self.actionQuit.triggered.connect(lambda: sys.exit(0))
        self.actionVerify.triggered.connect(self.verify_paths)

        self.run_default_scene()


    def run_default_scene(self):
        self.load_scene(default=True)
        self.load_solver(default=True)
        self.solver_dialog.solver_from_file(default=True)
        self.solver_dialog.choose_solver()
        #self.solve()


    def verify_paths(self):
        """
        Verify paths action
        """
        res, reason = verify_paths(self.discopygal_scene, self.paths)
        if res:
            msgbox = QtWidgets.QMessageBox(
                QtWidgets.QMessageBox.Icon.Information, 
                "Verify paths", "Successfully verified paths.")
            msgbox.exec()
        else:
            msgbox = QtWidgets.QMessageBox(
                QtWidgets.QMessageBox.Icon.Critical, 
                "Verify paths", "Paths are invalid: " + reason)
            msgbox.exec()

    def about_dialog(self):
        """
        Open the about dialog
        """
        dialog = QtWidgets.QDialog()
        dialog.ui = About_Dialog()
        dialog.ui.setupUi(dialog)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        dialog.setWindowTitle('About')
        dialog.exec_()

    def action_display_arrangement(self):
        """
        Display arrangement, if applicable
        """
        if self.solver_arrangement is None:
            return
        display_arrangement(self.solver_arrangement)
        
    def action_display_graph(self):
        """
        Display graph, if applicable
        """
        if len(self.solver_graph_vertices) > 0:
            self.clear_graph()
        else:
            self.show_graph()

    def show_graph(self):
        """
        Display the solver graph
        """ 
        if self.solver_graph is None:
            return

        for edge in self.solver_graph.edges:
            p, q = edge
            if type(p) is Point_2:
                x1, y1 = p.x().to_double(), p.y().to_double()
                x2, y2 = q.x().to_double(), q.y().to_double()
                self.solver_graph_vertices.append(
                    self.add_disc(DEFAULT_DISC_SIZE, x1, y1, QtCore.Qt.red, QtCore.Qt.red))
                self.solver_graph_vertices.append(
                    self.add_disc(DEFAULT_DISC_SIZE, x2, y2, QtCore.Qt.red, QtCore.Qt.red))
                self.solver_graph_edges.append(
                    self.add_segment(x1, y1, x2, y2, QtCore.Qt.red, opacity=0.7)
                )
            else:
                for i in range(p.dimension() // 2):
                    x1, y1 = p[2*i].to_double(), p[2*i+1].to_double()    
                    x2, y2 = q[2*i].to_double(), q[2*i+1].to_double()
                    self.solver_graph_vertices.append(
                    self.add_disc(
                        DEFAULT_DISC_SIZE, x1, y1, QtCore.Qt.blue, QtCore.Qt.red
                        )
                    )
                    self.solver_graph_vertices.append(
                        self.add_disc(
                            DEFAULT_DISC_SIZE, x2, y2, QtCore.Qt.blue, QtCore.Qt.red
                        )
                    )
                    if (x1,y1) != (x2,y2) :
                        self.solver_graph_edges.append(
                            self.add_segment(x1, y1, x2, y2, QtCore.Qt.blue, opacity=0.7)
                        )

    def clear_graph(self):
        """
        If a graph is drawn, clear it
        """
        for vertex in self.solver_graph_vertices:
            self.scene.removeItem(vertex.disc)
        for edge in self.solver_graph_edges:
            self.scene.removeItem(edge.line)
        self.solver_graph_vertices.clear()
        self.solver_graph_edges.clear()
        self.redraw()

    def action_pause(self):
        """
        Pause animation button
        """
        if self.is_queue_playing():
            self.pause_queue()
    
    def action_stop(self):
        """
        Stop the animation
        """
        if self.is_queue_playing() or self.is_queue_paused():
            self.stop_queue()
        # Move robots back to start
        self.scene_drawer.clear_scene()
        self.scene_drawer.draw_scene()

    def toggle_paths(self):
        """
        Toggle paths button
        """
        if len(self.path_vertices) > 0 or len(self.path_vetices_optimized) > 0:
            self.clear_paths()
        else:
            self.draw_both_paths()

    def anim_finished(self):
        """
        This is called when the animation is finished
        """
        pass

    def animate_paths(self):
        """
        Animate the paths (if exists)
        """
        if self.paths is None or len(self.paths.paths) == 0 or self.is_queue_playing():
            return

        # If we are just paused, resume
        if self.is_queue_paused():
            self.play_queue()
            return

        # Otherwise generate the paths for animation
        self.scene_drawer.clear_scene()
        self.scene_drawer.draw_scene()

        path_len = len(list(self.paths.paths.values())[
                       0].points)  # num of edges in paths
        animations = []
        for i in range(path_len-1):
            # All robots move in parallel along their edge
            animation_edges = []
            for robot in self.paths.paths:
                robot_gui = self.scene_drawer.robot_lut[robot][0]
                source = self.paths.paths[robot].points[i].location
                target = self.paths.paths[robot].points[i+1].location
                ix = source.x().to_double()
                iy = source.y().to_double()
                x = target.x().to_double()
                y = target.y().to_double()
                animation_edges.append(
                    self.linear_translation_animation(
                        robot_gui, ix, iy, x, y, 250
                    )
                )
            animations.append(self.parallel_animation(*animation_edges))

        self.queue_animation(*animations)
        self.play_queue()


    def draw_path(self, paths, vertices, edges, color:int):
        """
        Draw the paths (if exist)
        """
        if paths.paths is None:
            return

        for robot in paths.paths:
            points = paths.paths[robot].points
            for i in range(len(points)):
                x1, y1 = points[i].location.x().to_double(), points[i].location.y().to_double()
                vertices.append(self.add_disc(DEFAULT_DISC_SIZE, x1, y1, color, color))

                if i < len(points) - 1:
                    if points[i] == points[i + 1]:
                        continue
                    x2, y2 = points[i + 1].location.x().to_double(), points[i + 1].location.y().to_double()
                    if (x1,y1) != (x2,y2):
                        edges.append(self.add_segment( x1, y1, x2, y2,color))

    def draw_both_paths(self):
        """
        Draw both paths (if exist) (regular and optimized)
        """

        self.draw_path(self.paths_optimized, self.path_vetices_optimized, self.path_edges_optimized, QtCore.Qt.green)
        #self.draw_path(self.paths, self.path_vertices, self.path_edges, QtCore.Qt.magenta)



    def clear_paths(self):
        """
        Clear the paths if any were drawn
        """
        for vertex in set(self.path_vertices)|set(self.path_vetices_optimized):
            self.scene.removeItem(vertex.disc)
        for edge in set(self.path_edges)|set(self.path_edges_optimized):
            self.scene.removeItem(edge.line)
        self.path_vertices.clear()
        self.path_vetices_optimized.clear()
        self.path_edges.clear()
        self.path_edges_optimized.clear()

    def get_solver_args(self):
        """
        Extract a dict from the dynamically generated GUI arguments (to pass to the solver)
        """
        args = {}
        solver_args = self.solver_class.get_arguments()
        for arg in self.solver_gui_elements:
            if arg.endswith('_label'):
                continue
            _, _, ttype = solver_args[arg]
            args[arg] = ttype(self.solver_gui_elements[arg].text())
        return args

    def solve_thread(self):
        """
        The thread that is run by the "solve" function"
        """
        args = self.get_solver_args()
        solver = self.solver_class.from_arguments(args)
        solver.set_verbose(self.writer)
        solver.load_scene(self.discopygal_scene)
        #self.paths = solver.solve()
        self.paths, self.paths_optimized = solver.solve()
        self.solver_graph = solver.get_graph()
        self.solver_arrangement = solver.get_arrangement()

        self.solver = solver

    def solve(self):
        """
        This method is called by the solve button.
        Run the MP solver in parallel to the app.
        """
        if self.solver_class is None:
            return
        self.disable_toolbar()
        self.worker = Worker(self.solve_thread)
        self.worker.signals.finished.connect(self.solver_done)
        self.threadpool.start(self.worker)

    def disable_toolbar(self):
        """
        Disable icons on the toolbar while running
        """
        self.toolBar.setEnabled(False)

    def solver_done(self):
        """
        Enable icons on the toolbar after done running
        """
        self.toolBar.setEnabled(True)

    def select_solver(self, solver_name, default=False):
        """
        Set the selected solver.
        Also generate dynamically the GUI elements corresponding to the solver's arguments.
        """
        self.solver_class = globals()[solver_name]

        # Clear all elements
        for element in self.solver_gui_elements.values():
            element.setParent(None)
        self.solver_gui_elements.clear()

        # Generate the settings layout
        layout = QtWidgets.QVBoxLayout()
        args = self.solver_class.get_arguments()
        for arg, (description, default, _) in args.items():
            self.solver_gui_elements[arg +
                                     '_label'] = QtWidgets.QLabel(description)
            layout.addWidget(self.solver_gui_elements[arg + '_label'])
            self.solver_gui_elements[arg] = QtWidgets.QLineEdit(str(default))
            layout.addWidget(self.solver_gui_elements[arg])
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)

        # Attach layout to scroll widget
        self.scrollArea.setVerticalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.scrollArea.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setWidget(widget)

    def update_scene_metadata(self):
        """
        Update the scene's metadata
        """
        if 'version' in self.discopygal_scene.metadata:
            self.versionEdit.setText(self.discopygal_scene.metadata['version'])
        else:
            self.versionEdit.setText('NO VERSION')

        if 'solvers' in self.discopygal_scene.metadata:
            self.solversEdit.setText(self.discopygal_scene.metadata['solvers'])
        else:
            self.solversEdit.setText('NO SOLVERS')

        if 'details' in self.discopygal_scene.metadata:
            self.sceneDetailsEdit.setPlainText(self.discopygal_scene.metadata['details'])
        else:
            self.sceneDetailsEdit.setPlainText('NO DETAILS')

    def load_solver(self, default=False):
        """
        Open the "load solver" dialog
        """
        dialog = QtWidgets.QDialog()
        dialog.ui = SolverDialog(self, dialog, default=default)
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.solver_dialog = dialog.ui
        if not default:
            dialog.setWindowTitle('Open Solver...')
            dialog.exec_()

    def load_scene(self, default=False):
        """
        Load a scene.
        """
        if not default:
            name, _ = QtWidgets.QFileDialog.getOpenFileName(
                self.mainWindow, 'Load File')
            if name == '':
                return
        else:
            name = DEFAULT_SCENE
        self.clear()

        with open(name, 'r') as fp:
            d = json.load(fp)
        self.discopygal_scene = Scene.from_dict(d)
        self.scene_drawer.clear_scene()
        self.scene_drawer.scene = self.discopygal_scene
        self.scene_drawer.draw_scene()
        self.update_scene_metadata()

        self.clear_paths()
        if self.paths is not None:
            self.paths.paths.clear()
        self.paths = None

        self.scene_path = name
        self.scenePathEdit.setText(self.scene_path)

    def clear(self):
        """
        Clear scene, paths, graphs, etc.
        """
        self.scene_drawer.clear_scene()

        self.discopygal_scene = Scene()
        self.scene_drawer.scene = self.discopygal_scene
        self.scene_drawer.draw_scene()
        self.scene_path = ""
        self.clear_graph()
        self.clear_paths()
        if self.paths is not None:
            self.paths.paths.clear()
        self.paths = None
        self.scenePathEdit.setText(self.scene_path)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    gui = SolverViewerGUI()
    gui.mainWindow.show()
    sys.exit(app.exec_())
