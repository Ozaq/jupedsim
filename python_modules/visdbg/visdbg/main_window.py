import math
from pathlib import Path

import py_jupedsim
from PySide6.QtCore import QSettings, QSize
from PySide6.QtStateMachine import QFinalState, QState, QStateMachine
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QMainWindow,
    QMessageBox,
    QTabWidget,
)
from visdbg.geometry import Geometry
from visdbg.replay_widget import ReplayWidget
from visdbg.trajectory import Trajectory
from visdbg.view_geometry_widget import ViewGeometryWidget

from jupedsim.recording import Recording
from jupedsim.serialization import parse_wkt
from jupedsim.util import build_jps_geometry


class MainWindow(QMainWindow):
    def __init__(self, parent=None) -> None:
        QMainWindow.__init__(self, parent)
        self.settings = QSettings("jupedsim", "visdbg")
        self.setWindowTitle("visdbg")
        self._build_central_tabs_widget()
        self._build_menu_bar()
        self._build_state_machine()
        self.setVisible(True)

    def _build_central_tabs_widget(self):
        tabs = QTabWidget(self)
        tabs.setMinimumSize(QSize(640, 480))
        tabs.setMovable(True)
        tabs.setDocumentMode(True)
        tabs.setTabsClosable(True)
        tabs.setTabBarAutoHide(True)
        tabs.tabCloseRequested.connect(tabs.removeTab)
        self.setCentralWidget(tabs)
        self.tabs = tabs

    def _build_menu_bar(self) -> None:
        menu = self.menuBar()
        open_menu = menu.addMenu("File")
        open_wkt_act = open_menu.addAction("Open wkt file")
        open_wkt_act.triggered.connect(self._open_wkt)
        open_replay_act = open_menu.addAction("Open replay file")
        open_replay_act.triggered.connect(self._open_replay)

    def _build_state_machine(self) -> None:
        sm = QStateMachine(self)
        sm.finished.connect(QApplication.quit)

        start = self._build_start_state()
        sm.addState(start)

        exit = self._build_exit_state()
        sm.addState(exit)

        # start.addTransition(self.button.clicked, exit)

        sm.setInitialState(start)
        sm.start()
        self.state_machine = sm

    def _build_start_state(self) -> QState:
        state = QState()
        return state

    def _build_show_wkt_state(self) -> QState:
        state = QState()
        return state

    def _build_exit_state(self) -> QFinalState:
        state = QFinalState()
        return state

    def _open_wkt(self):
        base_path_obj = self.settings.value(
            "files/last_wkt_location",
            type=str,
            defaultValue=Path("~").expanduser(),
        )
        base_path = Path(str(base_path_obj))
        file, _ = QFileDialog.getOpenFileName(
            self, caption="Open WKT file", dir=str(base_path)
        )
        if not file:
            return
        file = Path(file)
        self.settings.setValue("files/last_wkt_location", str(file.parent))
        try:
            wkt = parse_wkt(Path(file).read_text(encoding="UTF-8"))
            navi = py_jupedsim.experimental.RoutingEngine(
                build_jps_geometry(wkt)
            )
            xmin, ymin, xmax, ymax = wkt.bounds
            info_text = f"Dimensions: {math.ceil(xmax - xmin)}m x {math.ceil(ymax-ymin)}m Triangles: {len(navi.mesh())}"
            name_text = f"Geometry: {file}"
            self.setUpdatesEnabled(False)
            geo = Geometry(navi)
            tab = ViewGeometryWidget(navi, geo, name_text, info_text)
            tab_idx = self.tabs.insertTab(0, tab, file.name)
            self.tabs.setCurrentIndex(tab_idx)
            self.setUpdatesEnabled(True)
        except Exception as e:
            QMessageBox.critical(
                self,
                "Error importing WKT geometry",
                f"Error importing WKT geometry:\n{e}",
            )
            return

    def _open_replay(self):
        base_path_obj = self.settings.value(
            "files/last_replay_location",
            type=str,
            defaultValue=Path("~").expanduser(),
        )
        base_path = Path(str(base_path_obj))
        file, _ = QFileDialog.getOpenFileName(
            self, caption="Open recording", dir=str(base_path)
        )
        if not file:
            return
        file = Path(file)
        self.settings.setValue("files/last_replay_location", str(file.parent))
        try:
            rec = Recording(file.as_posix())
            self.setUpdatesEnabled(False)
            navi = py_jupedsim.experimental.RoutingEngine(
                build_jps_geometry(rec.geometry())
            )
            geo = Geometry(navi)
            trajectory = Trajectory(rec)
            tab = ReplayWidget(navi, rec, geo, trajectory)
            tab_idx = self.tabs.insertTab(0, tab, file.name)
            self.tabs.setCurrentIndex(tab_idx)
            self.setUpdatesEnabled(True)
            self.update()
        except Exception as e:
            QMessageBox.critical(
                self,
                "Error importing simulation recording",
                f"Error importing simulation recording:\n{e}",
            )
            return
